#! /usr/bin/env python3
import rospy;


from pick_up_object_server import PickUpObjectAction;
from put_object_on_surface_server import PutObjectOnSurfaceAction;
from give_object_to_operator_server import GiveObjectToOperatorAction;
from receive_object_from_operator_server import ReceiveObjectFromOperatorAction;
from point_to_object_server import PointToObjectAction;
from find_placement_server import PlacementFinder;

if __name__ == '__main__':
    rospy.init_node("manipulation_node_general");

    if not rospy.has_param('use_grasping_synthesis'):
        rospy.logwarn("Waiting for rospy.param['use_grasping_synthesis']")
        rospy.sleep(5);
    if not rospy.has_param('use_collision_mapping'):
        rospy.logwarn("Waiting for rospy.param['use_collision_mapping']");
        rospy.sleep(5);

    pick_up_object = PickUpObjectAction(
        "pick_up_object", 
        use_collision_map=rospy.get_param('use_collision_mapping'), 
        use_grasp_synthesis=rospy.get_param('use_grasping_synthesis')
    )

    put_object_down = PutObjectOnSurfaceAction(
        "put_object_on_surface", use_collision_map=True,
    )

    give_object_to_operator = GiveObjectToOperatorAction(
        "give_object_to_operator", use_collision_map=False
    )

    recieve_object_from_operator = ReceiveObjectFromOperatorAction(
        "receive_object_from_operator", use_collision_map=False
    )

    point_to_object = PointToObjectAction("point_to_object")
    
    find_placement_loc = PlacementFinder()

    rospy.spin();