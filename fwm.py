from clover import srv
from std_srvs.srv import Trigger
import rospy
import math
import numpy as np
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
import tf.transformations
import matplotlib.pyplot as plt
import subprocess
import sys
import os

rospy.init_node('autonomous_flight')

# ==============================
# БЛОК ПРОКСИ СЕРВИСОВ И КОНФИГУРАЦИИ
# ==============================
# Service proxies
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

# Configuration parameters (Параметры конфигурации миссии)
WIREP_START = (2.0, 2.0)
WIREP_END = (2.0, 8.0)
WIREP_WIDTH = 0.05
FLIGHT_HEIGHT = 1.5
NUM_CHECKPOINTS = 5
TARGET_DISTANCE = 0.05
DISTANCE_TOLERANCE = 0.02
POSITION_TOLERANCE = 0.02
MAX_CORRECTION_SPEED = 0.15
THERMAL_TOPIC = "/thermal_camera/points"
THERMAL_THRESHOLD = 70.0
THERMAL_TOLERANCE = 2.0

# TF infrastructure
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
static_broadcaster = tf2_ros.StaticTransformBroadcaster()


# ==============================
# БЛОК НАСТРОЙКИ ТРАНСФОРМАЦИИ ДАННЫХ ДАТЧИКОВ
# ==============================
# Sensor transforms configuration
def setup_transforms():
    # Velodyne transform
    velodyne_transform = TransformStamped()
    velodyne_transform.header.frame_id = "base_link"
    velodyne_transform.child_frame_id = "velodyne"
    velodyne_transform.transform.translation.z = -0.02
    q = tf.transformations.quaternion_from_euler(1.571, 1.571, 0)
    velodyne_transform.transform.rotation.x = q[0]
    velodyne_transform.transform.rotation.y = q[1]
    velodyne_transform.transform.rotation.z = q[2]
    velodyne_transform.transform.rotation.w = q[3]

    # Thermal camera transform
    thermal_transform = TransformStamped()
    thermal_transform.header.frame_id = "base_link"
    thermal_transform.child_frame_id = "thermal_camera"
    thermal_transform.transform.translation.x = 0.055
    thermal_transform.transform.translation.y = -0.01
    thermal_transform.transform.translation.z = -0.03
    q_thermal = tf.transformations.quaternion_from_euler(1.571, 1.571, 0)
    thermal_transform.transform.rotation.x = q_thermal[0]
    thermal_transform.transform.rotation.y = q_thermal[1]
    thermal_transform.transform.rotation.z = q_thermal[2]
    thermal_transform.transform.rotation.w = q_thermal[3]

    static_broadcaster.sendTransform([velodyne_transform, thermal_transform])


# ==============================
# БЛОК ОБРАБОТКИ ТЕПЛОВИЗИОННЫХ ДАННЫХ
# ==============================
# Thermal data handling 
thermal_data = {
    'active': False,
    'points': None,
    'initialized': False
}

def thermal_callback(msg):
    if not thermal_data['active']:
        return
    
    try:
        transform = tf_buffer.lookup_transform('aruco_map', 'thermal_camera', rospy.Time(0))
        transformed_cloud = do_transform_cloud(msg, transform)
        
        points = pc2.read_points(transformed_cloud, 
                               field_names=("x", "y", "z", "temperature"),
                               skip_nans=True)
        
        valid_points = []
        for p in points:
            if THERMAL_THRESHOLD - THERMAL_TOLERANCE <= p[3] <= THERMAL_THRESHOLD + THERMAL_TOLERANCE:
                valid_points.append([p[0], p[1], p[2]])
        
        thermal_data['points'] = np.array(valid_points) if valid_points else None
        
    except Exception as e:
        log(f"Thermal processing error: {str(e)}", level='warn')


# ==============================
# БЛОК ВСПОМОГАТЕЛЬНЫХ ФУНКЦИЙ 
# ==============================
# Логирование данных через функцию
def log(message, level='info'):
    output = f"[FWM] {message}"
    print(output, flush=True)

# Генерация точек маршрута
def generate_checkpoints():
    return [(WIREP_START[0], y, FLIGHT_HEIGHT) 
            for y in np.linspace(WIREP_START[1], WIREP_END[1], NUM_CHECKPOINTS)]

# Генерация карты маршрута
def plot_flight_route(checkpoints, filename='/home/clover/Documents/p1/newroutes/flight_route.png'):

    try:
        plt.figure(figsize=(10, 6))
        
        # Извлекаем координаты X и Y
        x = [p[0] for p in checkpoints]
        y = [p[1] for p in checkpoints]
        
        # Рисуем маршрут и точки
        plt.plot(x, y, 'b-', linewidth=1.5, alpha=0.7)
        plt.scatter(x, y, c='red', s=50, marker='o', label='Route checkpoints')
        
        # Подписываем точки
        for i, (xp, yp) in enumerate(zip(x, y)):
            plt.text(xp, yp, f'WP{i+1}', fontsize=9, ha='right')

        plt.grid(True, linestyle='--', alpha=0.7)
        plt.legend()
        plt.axis('equal')
        
        # Сохраняем изображение
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        plt.close()
        
    except Exception as e:
        log(f"Error route: {str(e)}")


# ==============================
# БЛОК ФУНКЦИЙ НАВИГАЦИИ
# ==============================
def navigate_wait(x=0, y=0, z=0, speed=0.5, frame_id='body', auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=float('nan'), 
                  speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    if not res.success:
        raise Exception(res.message)
    
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x**2 + telem.y**2 + telem.z**2) < 0.2:
            return
        rospy.sleep(0.2)

def hold_position():
    telem = get_telemetry(frame_id='aruco_map')
    navigate(x=telem.x, y=telem.y, z=telem.z, frame_id='aruco_map', speed=0)
    log("Position hold active")


# ==============================
# БЛОК ОБРАБОТКИ ЛИДАРНЫХ ДАННЫХ
# ==============================
def process_point_cloud():
    try:
        # 1. Получение данных лидара
        cloud_msg = rospy.wait_for_message('/velodyne_points', PointCloud2, timeout=5)

        # 2. Преобразование координат в систему координат
        transform = tf_buffer.lookup_transform('aruco_map', 'velodyne', rospy.Time(0))

        # 3. Применение трансформации к облаку точек
        cloud = do_transform_cloud(cloud_msg, transform)

         # 4. Конвертация в массив точек (x,y,z)
        return np.array(list(pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)))
    except Exception as e:
        log(f"Point cloud processing error: {str(e)}")
        return None

def detect_wirep():
    points = process_point_cloud()
    if points is None or len(points) == 0:
        return None
    
    x_filter = (points[:,0] > WIREP_START[0] - WIREP_WIDTH/2) & \
              (points[:,0] < WIREP_START[0] + WIREP_WIDTH/2)
    filtered = points[x_filter]
    
    if len(filtered) == 0:
        return None
    
    hist, bins = np.histogram(filtered[:,2], bins=50)
    wirep_z = bins[np.argmax(hist)]
    
    z_filter = np.abs(filtered[:,2] - wirep_z) < 0.03
    height_filtered = filtered[z_filter]
    
    y_filter = (height_filtered[:,1] >= WIREP_START[1]) & \
               (height_filtered[:,1] <= WIREP_END[1])
    wirep_points = height_filtered[y_filter]
    
    if len(wirep_points) == 0:
        return None
    
    return (np.median(wirep_points[:,0]), 
            np.median(wirep_points[:,1]), 
            wirep_z)


# ==============================
# БЛОК ПОСАДОЧНОЙ ПОСЛЕДОВАТЕЛЬНОСТИ
# ==============================
def landing_sequence(target_pos):
    wirep_z = None
    log(f"Starting landing procedure: X={target_pos[0]:.3f} Y={target_pos[1]:.3f}")
    
    wirep_data = detect_wirep()
    if not wirep_data:
        log("WireP not detected! Aborting landing")
        return
    
    wirep_z = wirep_data[2]
    navigate_wait(x=target_pos[0], y=target_pos[1], z=wirep_z + TARGET_DISTANCE + 0.5, 
                 frame_id='aruco_map', speed=0.3)
    
    rate = rospy.Rate(15)
    landed = False
    
    while not landed and not rospy.is_shutdown():
        try:
            wirep_data = detect_wirep()
            if wirep_data:
                wirep_z = wirep_data[2]
                target_pos = (wirep_data[0], wirep_data[1], wirep_z + TARGET_DISTANCE)
            
            telem = get_telemetry(frame_id='aruco_map')
            
            dx = target_pos[0] - telem.x
            dy = target_pos[1] - telem.y
            
            if abs(dx) > POSITION_TOLERANCE or abs(dy) > POSITION_TOLERANCE:
                correction_speed = min(MAX_CORRECTION_SPEED, (abs(dx) + abs(dy)) * 0.5)
                navigate(
                    x=target_pos[0],
                    y=target_pos[1],
                    z=telem.z,
                    speed=correction_speed,
                    frame_id='aruco_map'
                )
                rospy.sleep(0.1)
                continue
            
            current_distance = telem.z - wirep_z
            
            if abs(current_distance - TARGET_DISTANCE) < DISTANCE_TOLERANCE:
                landed = True
                break
                
            vertical_speed = 0.07 if (current_distance - TARGET_DISTANCE) > 0.2 else 0.03
            target_z = wirep_z + TARGET_DISTANCE + (current_distance - TARGET_DISTANCE) * 0.3
            
            navigate(
                x=target_pos[0],
                y=target_pos[1],
                z=target_z,
                speed=vertical_speed,
                frame_id='aruco_map'
            )
            
            rate.sleep()
            
        except Exception as e:
            log(f"Landing error: {str(e)}")
            break

    hold_position()
    execute_post_landing_sequence()


# ==============================
# БЛОК ПОСЛЕПОСАДОЧНЫХ ОПЕРАЦИЙ
# ==============================
def execute_post_landing_sequence():
    log("Initiating post-landing procedures")
    try:
        script_sequence = ["TESTTASKWAIT5SEC.py", "homecoming.py"]
        
        for script in script_sequence:
            log(f"New task: {script}")
            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"
            
            process = subprocess.Popen(
                [sys.executable, "-u", script],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=0,
                env=env
            )
            
            from threading import Thread
            def output_reader(proc, name):
                with proc.stdout:
                    for line in proc.stdout:
                        log(f"[{name}] {line.strip()}")
            
            Thread(target=output_reader, args=(process, script)).start()
            
            if process.wait() != 0:
                raise RuntimeError(f"Script {script} failed")
                
        log("All tasks completed successfully")
        
    except Exception as e:
        log(f"Post-landing error: {str(e)}")
    finally:
        log("Mission complete")


# ==============================
# БЛОК ВЫПОЛНЕНИЯ ОСНОВНОЙ МИССИИ
# ==============================
def execute_mission():
    setup_transforms()
    thermal_sub = rospy.Subscriber(THERMAL_TOPIC, PointCloud2, thermal_callback)
    
    checkpoints = generate_checkpoints()
    log("Generated flight route:")

    plot_flight_route(checkpoints)

    for i, p in enumerate(checkpoints, 1):
        log(f"Waypoint {i}: X={p[0]:.2f} Y={p[1]:.2f} Z={p[2]:.2f}")

    navigate_wait(z=FLIGHT_HEIGHT, frame_id='body', auto_arm=True)
    first_waypoint, *scan_waypoints = checkpoints
    
    # Activate thermal after first waypoint
    navigate_wait(*first_waypoint, frame_id='aruco_map', speed=0.4)
    log("Scanners activation")
    thermal_data['active'] = True
    log("Starting wire scanning procedure")

    thermal_points = []
    wirep_data = []
    
    for wp in scan_waypoints:
        navigate_wait(*wp, frame_id='aruco_map', speed=0.3)
        
        # Collect thermal data
        if thermal_data['points'] is not None:
            thermal_points.extend(thermal_data['points'])
            thermal_data['points'] = None
        
        # Collect lidar data
        data = detect_wirep()
        if data:
            wirep_data.append(data)
    
    # Определение точки посадки
    if thermal_points:
        points_array = np.array(thermal_points)
        log(f"Selected thermal data points: {len(points_array)}")
        landing_point = (
            np.median(points_array[:,0]),
            np.median(points_array[:,1]),
            np.median(points_array[:,2]) + TARGET_DISTANCE
        )
    else:
        log("No thermal data available, using default selection")
        if not wirep_data:
            log("No sensor data available! Aborting")
            sys.exit(1)
            
        points_array = np.array(wirep_data)
        landing_point = (
            np.median(points_array[:,0]),
            np.median(points_array[:,1]),
            np.median(points_array[:,2]) + TARGET_DISTANCE
        )
    
    landing_sequence(landing_point)

if __name__ == '__main__':
    try:
        execute_mission()
    except Exception as e:
        log(f"Critical mission failure: {str(e)}")
        hold_position()
        rospy.sleep(5)
        rospy.signal_shutdown("Emergency shutdown")