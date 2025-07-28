import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
import numpy as np
import random
import argparse

"""
ПАРАМЕТРЫ PX4: EKF2_AID_MASK = 24
               EKF2_HGT_MODE = 3
"""

UAV_MODEL_NAME = 'clover'  # имя модели дрона в /gazebo/model_states

class UWBSimulator:
    def __init__(self, noise_std_xy, noise_std_z, rate_hz, latency_mean, drop_rate):
        """
        Инициализация узла ROS и параметров симулятора UWB
        :param noise_std_xy: стандартное отклонение шума по осям X и Y (м)
        :param noise_std_z: стандартное отклонение шума по оси Z (м)
        :param rate_hz: частота публикации сообщений (Гц)
        :param latency_mean: средняя задержка публикации (сек)
        :param drop_rate: вероятность пропуска сообщения (0.0 - 1.0)
        """
        rospy.init_node('uwb_sim_node', anonymous=True)

        self.noise_std_xy = noise_std_xy
        self.noise_std_z = noise_std_z
        self.rate_hz = rate_hz
        self.latency_mean = latency_mean
        self.drop_rate = drop_rate

        self.pose = None

        # Паблишер для публикации зашумленной позиции в формате PoseStamped
        self.pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)

        # Подписка на топик с состояниями моделей Gazebo для получения истинной позиции дрона
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

        # Таймер для периодической публикации
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.timer_callback)

        rospy.loginfo("UWB simulator started for model '%s'", UAV_MODEL_NAME)
        rospy.loginfo("Noise XY=%.3f, Z=%.3f | Rate=%.1fHz | Latency=%.3fs | DropRate=%.2f",
                      self.noise_std_xy, self.noise_std_z, self.rate_hz, self.latency_mean, self.drop_rate)

    def model_states_callback(self, msg):
        """
        Callback для топика /gazebo/model_states
        Получаем индекс модели по имени и сохраняем текущую позу
        """
        try:
            idx = msg.name.index(UAV_MODEL_NAME)
            self.pose = msg.pose[idx]
        except ValueError:
            # Предупреждение если модель не найдена
            rospy.logwarn_once("Model '%s' not found in /gazebo/model_states", UAV_MODEL_NAME)

    def add_noise(self, pose):
        """
        Добавляет шум к позиции в pose
        :param pose: geometry_msgs/Pose
        :return: PoseStamped с шумом
        """
        noisy = PoseStamped()
        noisy.header.stamp = rospy.Time.now()
        noisy.header.frame_id = "map"

        # Добавляем шум к позиции
        noisy.pose.position.x = pose.position.x + np.random.normal(0, self.noise_std_xy)
        noisy.pose.position.y = pose.position.y + np.random.normal(0, self.noise_std_xy)
        noisy.pose.position.z = pose.position.z + np.random.normal(0, self.noise_std_z)

        # Копируем ориентацию
        noisy.pose.orientation = pose.orientation
        return noisy

    def timer_callback(self, event):
        """
        Периодический вызов по таймеру для публикации зашумленной позиции
        Реализует задержку и случайный пропуск сообщений
        """
        if self.pose is None:
            return

        # Имитация пропуска сообщений с вероятностью drop_rate
        if random.random() < self.drop_rate:
            rospy.logdebug("UWB message dropped (simulated)")
            return

        # Добавляем шум к позиции
        noisy_pose = self.add_noise(self.pose)

        # Генерируем задержку публикации с экспоненциальным распределением
        latency = np.random.exponential(self.latency_mean)

        # Публикация сообщения с задержкой latency
        rospy.Timer(rospy.Duration(latency), lambda _: self.pub.publish(noisy_pose), oneshot=True)


def parse_args():
    """
    Аргументы для конфигурации симулятора
    """
    parser = argparse.ArgumentParser(description="Simulate UWB-based vision_pose publisher for PX4")
    parser.add_argument('--noise_std_xy', type=float, default=0.05, help='XY position noise std-dev (m)')
    parser.add_argument('--noise_std_z', type=float, default=0.1, help='Z position noise std-dev (m)')
    parser.add_argument('--rate', type=float, default=20.0, help='Publishing rate (Hz)')
    parser.add_argument('--latency_mean', type=float, default=0.05, help='Average publishing latency (s)')
    parser.add_argument('--drop_rate', type=float, default=0.05, help='Probability of message drop (0.0 - 1.0)')
    return parser.parse_args()


if __name__ == '__main__':
    try:
        args = parse_args()
        UWBSimulator(args.noise_std_xy, args.noise_std_z, args.rate, args.latency_mean, args.drop_rate)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
