from botX.components import BaseComponent
from botX.applications import external_command_pool
import rospy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, PointCloud2
import time

class GazeboAPI(BaseComponent):

    def setup(self):
        """
        download .gazebo from github url and put it in absolute path
        """
        command = 'roslaunch haptica_gazebo demo_world.launch'
        self.proc_id = external_command_pool.start_command(command)
        """
        wait until it is lauched
        """
        self.buf = []
        rospy.init_node('listener')
        rospy.Subscriber("clock", Clock, self.cache_info)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.cache_info)
        rospy.Subscriber("/camera/depth/points", Image, self.cache_info)
        """
        wait until it is receiving
        """

    def cache_info(self, msg):
        # print("Message: ", msg)
        self.buf.append([msg, type(msg)])
        return

    def get_environment_status(self):
        """
        preprocess it to usable format
        """
        return self.buf[-1:]

    def get_image(self):
        image = [x[0] for x in self.buf if x[1] is Image]
        
        # Intialize the image_saver node
        # image_saver = 'rosrun image_view image_saver image:=/camera/depth/image_raw _save_all_image:=false _encoding:=32UF1 _filename_format:=foo.tiff __name:=image_saver'
        # self.image_saver_id = external_command_pool.start_command(image_saver)

        image_viewer = 'rosrun image_view image_view image:=/camera/image_raw __name:=image_viewer'
        self.image_viewer_id = external_command_pool.start_command(image_viewer)

        # depth_image_saver = 'rosrun image_view image_saver image:=/camera/depth/image_raw _save_all_image:=false _filename_format:=foo_depth.jpg __name:=depth_image_saver'
        # self.depth_image_saver_id = external_command_pool.start_command(depth_image_saver)
        depth_image_viewer = 'rosrun image_view image_view image:=/camera/depth/image_raw __name:=depth_image_viewer'
        self.depth_image_viewer_id = external_command_pool.start_command(depth_image_viewer)
        
        # Pause to allow the image_saver node to run
        time.sleep(1)

        # Call to save a single image 
        # save_image = 'rosservice call /image_saver/save'
        # self.save_image_id = external_command_pool.start_command(save_image)
        print("getting image...")

        # Pause to allow image to save before process gets shut down
        time.sleep(1)

        # save_depth_image = 'rosservice call /depth_image_saver/save'
        # self.save_depth_image_id = external_command_pool.start_command(save_depth_image)
        # print("getting depth image...")
        
        # Pause to allow image to save before process gets shut down
        time.sleep(2)

        return image[-1:]

    def bag_vision(self, duration):
        bag_cmd = 'rosbag record -O vision /camera/image_raw /camera/depth/image_raw /camera/depth/points'
        bag_cmd_id = external_command_pool.start_command(bag_cmd)
        print("bagging vision data...")
        time.sleep(duration)
        external_command_pool.end_command(bag_cmd_id)

        bag_reindex_cmd = 'rosbag reindex vision.bag.active'
        bag_reindex_id = external_command_pool.start_command(bag_reindex_cmd)
        time.sleep(1)

        return


    def get_clock(self):
        
        pass

    def shutdown(self):
        external_command_pool.end_command(self.proc_id)
        external_command_pool.end_command(self.image_viewer_id)
        external_command_pool.end_command(self.depth_image_viewer_id)

        # external_command_pool.end_command(self.save_image_id)
        # external_command_pool.end_command(self.image_saver_id)
        # external_command_pool.end_command(self.save_depth_image_id)
        # external_command_pool.end_command(self.depth_image_saver_id)
