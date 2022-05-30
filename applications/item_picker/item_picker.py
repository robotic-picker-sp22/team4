#!/usr/bin/env python 

from applications.scripts.auto_picker import AutoPicker
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import perception
import rospy


def wait_for_time(): 
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

    
def main():                                                                             
    rospy.init_node('item_picker')
    wait_for_time()                                                                     
    pub = rospy.Publisher('add_item', String, queue_size=1)       
    rate = rospy.Rate(2)
    ap = AutoPicker([], debug=True)
    print("Beginning program...\nType quit to quit\n")
    while True:
        print("Select which object to pick:", list(objects.keys()))
        print(">", end=" ")
        obj_name = input()

        if obj_name == "quit":
            break
        elif obj_name not in ap.objects:
            print("Object " + obj_name + " does not exist")
        else:
            print("-----------------------------------")
            ap.pick_object(obj_name, *objects[obj_name])
            print("-----------------------------------")
            
    
    
if __name__ == '__main__':
    main()