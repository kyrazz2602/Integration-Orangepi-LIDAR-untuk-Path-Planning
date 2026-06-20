#!/usr/bin/env python3
"""
Room Map Manager Node
Menangani permintaan simpan peta per-ruangan dari dashboard web,
memanggil service slam_toolbox untuk serialize/save map, lalu reset
untuk mulai scan ruangan berikutnya.
"""

import os
import sys
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Try importing slam_toolbox services with fallback
try:
    from slam_toolbox.srv import SaveMap, Reset
    RESET_SERVICE_AVAILABLE = True
except ImportError:
    try:
        from slam_toolbox.srv import SaveMap
        RESET_SERVICE_AVAILABLE = False
        Reset = None
    except ImportError:
        print("ERROR: slam_toolbox services not found!")
        sys.exit(1)

try:
    import firebase_admin
    from firebase_admin import db
    from firebase_admin import credentials
except ImportError:
    print("ERROR: firebase-admin not installed. Run: pip install firebase-admin")
    sys.exit(1)


class RoomMapManager(Node):
    def __init__(self):
        super().__init__('room_map_manager')
        
        # Declare Parameters
        self.declare_parameter('maps_base_dir', os.path.expanduser('~/maps'))
        self.declare_parameter('firebase_db_url', 'https://airguard-b7ef4-default-rtdb.asia-southeast1.firebasedatabase.app')
        
        self.maps_base_dir = self.get_parameter('maps_base_dir').value
        self.firebase_db_url = self.get_parameter('firebase_db_url').value

        # Initialize clients for slam_toolbox services
        self.save_map_client = self.create_client(SaveMap, '/slam_toolbox/save_map')
        
        if RESET_SERVICE_AVAILABLE:
            self.reset_client = self.create_client(Reset, '/slam_toolbox/reset')
        else:
            self.reset_client = None
            self.get_logger().warn("Service /slam_toolbox/reset (Reset.srv) tidak terdeteksi di modul Python ini.")

        # Initialize Firebase
        self.db_ref = None
        self.firebase_ready = self._init_firebase()

        if self.firebase_ready and self.db_ref:
            self.command_ref = self.db_ref.child('Command')
            # Start Firebase Listener in a separate daemon thread
            self.listener_thread = threading.Thread(target=self._listen_save_requests, daemon=True)
            self.listener_thread.start()
            self.get_logger().info('Room Map Manager siap mendengarkan permintaan simpan peta.')
        else:
            self.get_logger().error('Gagal menghubungkan Room Map Manager ke Firebase!')

    def _init_firebase(self) -> bool:
        try:
            if firebase_admin._apps:
                self.db_ref = db.reference()
                return True

            cred_filename = 'airguard-b7ef4-firebase-adminsdk-fbsvc-6361f49d51.json'
            script_dir = os.path.dirname(os.path.abspath(__file__))
            
            paths_to_check = [
                os.path.join(script_dir, cred_filename),
                os.path.join(os.getcwd(), cred_filename),
                os.path.join(os.path.expanduser('~'), cred_filename),
            ]
            
            cred_path = None
            for path in paths_to_check:
                if os.path.exists(path):
                    cred_path = path
                    break
                    
            if not cred_path:
                self.get_logger().error(f'Firebase credentials file {cred_filename} not found in paths: {paths_to_check}')
                return False

            self.get_logger().info(f'Using Firebase credentials from: {cred_path}')
            cred = credentials.Certificate(cred_path)
            firebase_admin.initialize_app(cred, {'databaseURL': self.firebase_db_url})
            self.db_ref = db.reference()
            return True
        except Exception as e:
            self.get_logger().error(f'Firebase init error: {e}')
            return False

    def _listen_save_requests(self):
        try:
            self.command_ref.child('save_map_request').listen(self._on_save_request)
        except Exception as e:
            self.get_logger().error(f"Error starting Firebase save request listener: {e}")

    def _on_save_request(self, event):
        data = event.data
        if not data or 'room_name' not in data:
            return
            
        room_name = str(data['room_name']).strip()
        house_name = str(data.get('house_name', 'default')).strip()
        
        if not room_name:
            return

        self.get_logger().info(f'Menerima permintaan simpan peta: Rumah: "{house_name}", Ruangan: "{room_name}"')
        
        # Kirim status "pending"
        self._report_status(house_name, room_name, 'pending', 'Memulai penyimpanan peta...')
        
        # Jalankan proses penyimpanan peta
        self.save_current_map(house_name, room_name)
        
        # Bersihkan data command setelah diproses
        self.command_ref.child('save_map_request').delete()

    def save_current_map(self, house_name: str, room_name: str):
        target_dir = os.path.join(self.maps_base_dir, house_name)
        
        try:
            os.makedirs(target_dir, exist_ok=True)
        except Exception as e:
            self.get_logger().error(f"Gagal membuat direktori {target_dir}: {e}")
            self._report_status(house_name, room_name, 'error', f'Gagal membuat direktori lokal: {e}')
            return

        # Suffix file absolute path tanpa ekstensi (slam_toolbox otomatis menambahkan .pgm & .yaml)
        file_stem = os.path.join(target_dir, room_name)

        if not self.save_map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/slam_toolbox/save_map service tidak tersedia!')
            self._report_status(house_name, room_name, 'error', 'Service /slam_toolbox/save_map tidak tersedia')
            return

        req = SaveMap.Request()
        req.name.data = file_stem
        
        self.get_logger().info(f"Memanggil service save_map untuk {file_stem}...")
        future = self.save_map_client.call_async(req)
        
        # Threaded callback handler
        future.add_done_callback(
            lambda f: self._on_save_done(f, house_name, room_name, file_stem)
        )

    def _on_save_done(self, future, house_name, room_name, file_stem):
        try:
            result = future.result()
            # Nilai sukses biasanya adalah 0 (RESULT_SUCCESS)
            if result.result == 0:
                self.get_logger().info(f'Peta berhasil disimpan ke: {file_stem}')
                self._report_status(house_name, room_name, 'success', f'Peta disimpan di {file_stem}')
                
                # Reset slam_toolbox agar siap untuk scan ruangan berikutnya
                self.reset_slam(house_name, room_name)
            else:
                self.get_logger().error(f'Gagal menyimpan peta, result code: {result.result}')
                self._report_status(house_name, room_name, 'error', f'Gagal menyimpan peta (code: {result.result})')
        except Exception as e:
            self.get_logger().error(f'Error menyelesaikan pemanggilan save_map: {e}')
            self._report_status(house_name, room_name, 'error', f'Error memproses penyimpanan: {e}')

    def reset_slam(self, house_name: str, room_name: str):
        if not self.reset_client:
            self.get_logger().warn('Reset SLAM dilewati karena service reset tidak tersedia di sistem ini.')
            return

        if not self.reset_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('Service /slam_toolbox/reset tidak merespons, lewati reset otomatis.')
            return

        req = Reset.Request()
        self.get_logger().info('Memanggil service /slam_toolbox/reset...')
        reset_future = self.reset_client.call_async(req)
        reset_future.add_done_callback(
            lambda f: self.get_logger().info('Service reset selesai dipanggil.')
        )

    def _report_status(self, house_name: str, room_name: str, status: str, info: str):
        if not self.db_ref:
            return
        
        try:
            self.db_ref.child('SavedMaps').child(house_name).child(room_name).set({
                'status': status,
                'info': info,
                'timestamp': datetime.now().isoformat(),
            })
        except Exception as e:
            self.get_logger().error(f'Gagal mengirim status simpan ke Firebase: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RoomMapManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
