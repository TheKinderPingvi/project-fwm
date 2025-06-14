import eventlet
eventlet.monkey_patch()

from flask import Flask, render_template
from flask_socketio import SocketIO
import subprocess
import os
import signal
import threading
import rospy
from rosgraph_msgs.msg import Log as RosLog

# Инициализация Flask приложения и SocketIO
app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(
    app,
    async_mode='eventlet',
    cors_allowed_origins="*",
    logger=True,
    engineio_logger=True
)

# Глобальные переменные и константы
processes = {}
process_lock = threading.Lock()
ALLOWED_CATEGORIES = {'FWM', 'HOME', 'THERMAL', 'NAV'}  

# Класс для прослушивания ROS логов
class ROSListener(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.stop_flag = threading.Event()
        self.allowed_nodes = {'/autonomous_flight'}  

    def run(self):
        try:
            rospy.init_node('web_interface', anonymous=True)
            rospy.Subscriber('/rosout_agg', RosLog, self.ros_log_callback)
            while not self.stop_flag.is_set() and not rospy.is_shutdown():
                rospy.sleep(0.1)
        except Exception as e:
            self.send_log(f'ROS Error: {str(e)}', 'SYSTEM')

    def ros_log_callback(self, msg):
        try:
            if msg.name not in self.allowed_nodes:
                return
                
            levels = {1: 'DEBUG', 2: 'INFO', 4: 'WARN', 8: 'ERROR'}
            level = levels.get(msg.level, 'UNKNOWN')
            self.send_log(msg.msg, level)
        except Exception as e:
            pass

    def send_log(self, message, category='SYSTEM'):
        if category not in ALLOWED_CATEGORIES:
            return
            
        with app.app_context():
            socketio.emit('log', {
                'data': message,
                'category': category
            })

ros_listener = ROSListener()

# Роутинг Flask
@app.route('/')
def index():
    return render_template('index.html')

# Обработчики событий SocketIO
@socketio.on('connect')
def handle_connect():
    socketio.emit('log', {
        'data': 'Connected to Clover COEX Control System',
        'category': 'SYSTEM'
    })

@socketio.on('start_mission')
def handle_start_mission():
    with process_lock:
        if 'fwm' not in processes or processes['fwm'].poll() is not None:
            start_process('fwm', 'fwm.py')

@socketio.on('emergency_home')
def handle_emergency_home():
    with process_lock:
        stop_all_processes()
        start_process('homecoming', 'homecoming.py')

# Функции управления процессами
def start_process(name, script):
    if name in processes and processes[name].poll() is None:
        return

    env = os.environ.copy()
    env.update({
        'PYTHONUNBUFFERED': '1',
        'PYTHONIOENCODING': 'utf-8'
    })

    try:
        processes[name] = subprocess.Popen(
            ['python3', '-u', script],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=0,
            env=env,
            preexec_fn=os.setsid
        )
        socketio.start_background_task(log_reader, processes[name], name)
    except Exception as e:
        send_log(f'Failed to start {name}: {str(e)}', 'SYSTEM')

def stop_all_processes():
    for name in list(processes.keys()):
        proc = processes.pop(name)
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass

# Функция чтения логов процессов
def log_reader(process, name):
    while True:
        line = process.stdout.readline()
        if not line:
            if process.poll() is not None:
                break
            continue
        try:
            cleaned_line = line.strip()
            if cleaned_line:
                if cleaned_line.startswith('[') and ']' in cleaned_line:
                    end_idx = cleaned_line.find(']')
                    category = cleaned_line[1:end_idx].strip()
                    message = cleaned_line[end_idx+1:].strip()
                    
                    if category in ALLOWED_CATEGORIES:
                        send_log(message, category)
                else:
                    send_log(cleaned_line, name.upper())
        except Exception as e:
            pass

# функция отправки логов
def send_log(message, category):
    with app.app_context():
        socketio.emit('log', {
            'data': message,
            'category': category
        })

# Запуск web-interface
if __name__ == '__main__':
    ros_listener.start()
    socketio.run(
        app,
        host='0.0.0.0',
        port=5000,
        debug=False,
        use_reloader=False,
        log_output=True
    )