from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node
from pm_msgs.srv import EmptyWithSuccess, CreateVizAdhesivePoint
from pm_msgs.msg import VizAdhesivePoint,VizAdhesivePoints
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Pose



def adapt_transform_for_new_parent_frame(child_frame, new_parent_frame, tf_buffer: Buffer)->Pose:
    # this function adapts the tf for parent_frame changes
    try: 
        t:TransformStamped = tf_buffer.lookup_transform(new_parent_frame, child_frame,rclpy.time.Time())


        result = Pose()
        result.position.x = t.transform.translation.x
        result.position.y = t.transform.translation.y
        result.position.z = t.transform.translation.z

        result.orientation.x = t.transform.rotation.x
        result.orientation.y = t.transform.rotation.y
        result.orientation.z = t.transform.rotation.z
        result.orientation.w = t.transform.rotation.w

        return result
    
    except Exception as e:
        print(f"Frame '{child_frame}' does not exist in TF! {str(e)}")
        return None

    

def get_transform_for_frame_in_world(frame_name: str, tf_buffer: Buffer, logger = None) -> TransformStamped:
    # this function adapts the tf for parent_frame changes
    #transform:TransformStamped = tf_buffer.lookup_transform(frame_name, 'world',rclpy.time.Time())
    try:
        
        transform:TransformStamped = tf_buffer.lookup_transform('world', frame_name, rclpy.time.Time(),rclpy.duration.Duration(seconds=1.0))
        if logger is not None:
            logger.debug(f"Frame '{frame_name}' found in TF!")
    except Exception as e:
        transform = None
        raise ValueError(f"Frame '{frame_name}' does not exist in TF! {str(e)}")
    return transform

def get_transform_for_frame(frame_name: str, parent_frame:str, tf_buffer: Buffer, logger = None) -> TransformStamped:
    # this function adapts the tf for parent_frame changes
    #transform:TransformStamped = tf_buffer.lookup_transform(frame_name, 'world',rclpy.time.Time())
    try:
        
        transform:TransformStamped = tf_buffer.lookup_transform(parent_frame, frame_name, rclpy.time.Time(),rclpy.duration.Duration(seconds=1.0))
        if logger is not None:
            logger.debug(f"Frame '{frame_name}' found in TF!")
    except Exception as e:
        transform = None
        raise ValueError(f"Frame '{frame_name}' does not exist in TF! {str(e)}")
    return transform


class AdhesiveDisplay(Node):

    def __init__(self):
        super().__init__('adhesive_display_node')
        #self.adhesive_points:list[VizAdhesivePoint] = []

        self.adhesive_points = VizAdhesivePoints()

        self.clear_srv = self.create_service(EmptyWithSuccess, self.get_name()+'/clear_points', self.clear_callback)

        # create publisher
        self.publisher_ = self.create_publisher(VizAdhesivePoints, self.get_name()+'/adhesive_points', 10)

        self.clear_srv = self.create_service(CreateVizAdhesivePoint, self.get_name()+'/add_point', self.add_adhesive_point)

        # create timer
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.marker_publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.logger = self.get_logger()
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

    def add_adhesive_point(self, request:CreateVizAdhesivePoint.Request, response:CreateVizAdhesivePoint.Response):

        transform = adapt_transform_for_new_parent_frame(child_frame = request.point.endeffector_frame, 
                                                          new_parent_frame = request.point.parent_frame, 
                                                          tf_buffer=self.tf_buffer)
        
        if transform is None:
            response.success = False
            self.logger.error(f"Could not adapt transform for frame '{request.point.endeffector_frame}'")
            return response

        #transform = get_transform_for_frame_in_world(request.point.parent_frame, tf_buffer=self.tf_buffer, logger=self.logger)

        request.point.point_pose.position.x = transform.position.x + request.point.point_pose.position.x
        request.point.point_pose.position.y = transform.position.y + request.point.point_pose.position.y
        request.point.point_pose.position.z = transform.position.z + request.point.point_pose.position.z - (request.point.hight/1000)/2

        self.adhesive_points.points.append(request.point)
        response.success = True
        return response

    def clear_callback(self, request, response:EmptyWithSuccess.Response):

        self.adhesive_points.points.clear()
        self.get_logger().info('Clearing points')
        response.success = True

        return response

    def timer_callback(self):
        self.publisher_.publish(self.adhesive_points)
        self.publish_adhesive_marker()

    def publish_adhesive_marker(self):
        self.delete_all_markers()
        marker_array = MarkerArray()

        for index, adhesive_point in enumerate(self.adhesive_points.points):
            adhesive_point:VizAdhesivePoint
            marker = self.marker_from_adhesive_point(adhesive_point, index)
            marker_array.markers.append(marker)
            self.logger.warn(f"Publishing marker for {adhesive_point.parent_frame}")
            self.logger.warn(f"Transform marker for {str(adhesive_point.point_pose)}")
            #self.logger.warn(f"Publishing marker for {tolerance_handle.frame_name}")

        self.logger.info(f"Published {len(marker_array.markers)} markers.")
        self.marker_publisher.publish(marker_array)

    def delete_all_markers(self):
        marker_array = MarkerArray()
        
        # Replace with the range of IDs you want to delete
        for i in range(len(self.adhesive_points.points)):  
            marker = Marker()
            #marker.header.frame_id = "world"
            marker.id = i
            marker.ns = "adhesive_points"
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)
        
        self.marker_publisher.publish(marker_array)
        self.get_logger().info("Published DELETE action to remove all markers.")

    def marker_from_adhesive_point(self, adhesive_point:VizAdhesivePoint, id:int):

        marker = Marker()
        
        marker.header.frame_id = adhesive_point.parent_frame
        marker.header.stamp = rclpy.time.Time().to_msg()
        marker.ns = 'adhesive_points'
        marker.id = id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x =  adhesive_point.point_pose.position.x
        marker.pose.position.y =  adhesive_point.point_pose.position.y
        marker.pose.position.z = adhesive_point.point_pose.position.z 

        marker.scale.x = adhesive_point.diameter/1000
        marker.scale.y = adhesive_point.diameter/1000
        marker.scale.z = adhesive_point.hight/1000

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        return marker


def main(args=None):
    rclpy.init(args=args)

    minimal_service = AdhesiveDisplay()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()