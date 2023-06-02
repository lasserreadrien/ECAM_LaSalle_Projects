# GarbageCollector Project S8
These are the files used for the GarbageCollector project of my 8th Semester at ECAM LaSalle.

## Choose_from_fixed_positions.py

For that script, there are three functions being *‘talker’*, *‘calback_func’* and the main one.
For the *‘talker’*, the definition of the parameters is done as such:

rospy.init_node('choose_from_fixed_positions') #setting up the node
pub = rospy.Publisher('set_position', SetPosition, queue_size=100)#setting up the Publisher
rospy.Subscriber('choose_positions', FixedChoice, callback_func) #setting up the Subscriber

By that, it can be understood that the node will subscribe to the */choose_positions* topic, waiting for a *FixedChoice* message to be used in the *‘callback_func’*. It will then publish information of *SetPosition* type to the */set_position* topic. The node is called *‘choose_from_fixed_positions’* as for the name of the executable.
In the *‘callback_func’*, it uses data as an input, and when looking at the *FixedChoice.msg* file in the msg folder, it can be understood that to get the integer value, the following statement is used:

data.choice

Then, in this callback function, the information is used with a simple *if, elif, else* statement, with every cases, with integers from 1 to 6 included, and the else returning the 1st position. In every *if/elif* statement, the choice is checked, and if true, the python list *desired_values* is updated with the corresponding values. Example:

desired_values = [1, 650, 2, 440, 3, 430, 4, 545]  #arm perfectly horizontal

In that, the list can be detailed as [ID1, Position1, ID2, Position2, …, ID4, Position4]. In fact, the desired position for each motor (with their IDs) is stored in the right order.
When exiting that callback function, it will comeback to the *‘talker’* function and send the messages to the desired topic for each motor. In fact, as the *read_write_node.py* file only cares for one ID and one position, our node will just publish four different messages, for four updates of the motors. 
For example, to update the first motor, these lines will be used:

pub.publish(desired_values[0], desired_values[1])
r.sleep()

The publication will take into arguments the two data needed for the specific *SetPosition.msg*. Meaning, that it will take the first ID and the first position for the first motor, and the same for the other ones. It sleeps following a 10Hz rate, defined a bit before in the code.
When used, this whole script achieved to work when ran at the same time as the *read_write_node.py*. The team just had to publish a message on the topic */choose_positions* of the following type:

Rostopic pub /choose_positions dynamixel_sdk_examples_2/FixedChoice “{choice: 1}”

When re-attaching the arm to the Robotino, and trying to access it with the Jetson Nano, the team updated the *read_write_node.py* on the Jetson with the right information, created the *FixedChoice.msg* in the msg folder, and created the *choose_from_fixed_positions.py* node also, with a simple copy and paste of the work previously done.
From that, they were able to launch the two nodes (with the motor_position.launch launch file, updated for the right nodes) at the same time and make it work. For that they had to use the right USB port of the Jetson Nano (USB1 or USB0) in the code.

The next step is to re-assess the desired positions when attached to the Robotino, to have precise values for the desired positions and update the node code. This has been done in the additional sessions.

As said in the Arm displacement section, the new_test.py executable has been created as follows.

## New_test.py

For the *‘talker’*, the definition of the parameters is done as such:

rospy.init_node('new_test') #setting up the node
pub = rospy.Publisher('choose_positions', FixedChoice, queue_size=10) #publisher
rospy.Subscriber('scheme', FixedChoice, callback_fun) #subscriber

By that, it can be understood that the node will subscribe to the */scheme* topic, waiting for a *FixedChoice* message to be used in the *‘callback_fun’*. It will then publish information of FixedChoice type to the */choose_positions* topic. The node is called *‘new_test’* as for the name of the executable.
In the *‘callback_fun’*, it uses data as an input, and when looking at the *FixedChoice.msg* file in the msg folder, it can be understood that to get the integer value, the following statement is used:

data.choice

Then, in this callback function, the information is used with a simple *if, else* statement, that will check if the new value is already the same as the previous one or not. If it already is the same, the boolean variable called bol would be set to *True*, and *False* in the other case. Then, the temporary variable is updated with the new value. Example:

temporary=data.choice

To be noted that both *bol* and *temporary* are defined as global variables, to be then re-used in the *‘talker’* function. When exiting that callback function, it will comeback to the *‘talker’* function and send the messages to the desired topic following a special *if* statement. In fact, in the *‘talker’*, the status of the *bol* variable will be checked, and if it is *False*, meaning that the current value read by the callback function is different than the previous one, it would actuate with the desired scheme, the arm motors. This means, that at first, the False being the initial value of the *bol* and *temporary* being set to 2, it will actuate the motors with the second scheme until the value of *bol* is changed to *True*, or again to *False*. 
This makes it obvious that, as long as the subscribed topic */scheme* is not sending any data, the *bol* variable will stay to *False* and will keep on actuating the motors with the second scheme, opening and closing the gripper while this following statement is true:

not rospy.is_shutdown()

Therefore, the *demonstration.py* node has to be launched to stop that constant actuation. Indeed, this executable publishes on the */scheme* topic values of *FixedChoice* type, with either a 1 or 2, for the specific scenarii.
For the *‘talker’* function, it will publish the information to the published topic */choose_positions* as follows (example with first scheme):

if temporary==1:
				pub.publish(1) #arm horizontal, open
				r.sleep()
				pub.publish(3) #arm down, open
				r.sleep()
				pub.publish(4) #arm down, closed
				r.sleep()
				pub.publish(2) #arm horizontal, closed
        
It sleeps following a 1Hz rate, defined a bit before in the code.

When used along with the previous nodes, this whole script achieved to work. The team just had to publish messages on the topic */scheme* of the following type:

rostopic pub /scheme dynamixel_sdk_examples_2/FixedChoice “{choice: 2}”
rostopic pub /scheme dynamixel_sdk_examples_2/FixedChoice “{choice: 1}”

## Demonstration.py

For the *‘talker’*, the definition of the parameters is done as such:

rospy.init_node('demonstration')
pub=rospy.Publisher('scheme', FixedChoice, queue_size=10)
rospy.Subscriber('choose_archi', Bool, clb)

By that, it can be understood that the node will subscribe to the */choose_archi* topic, waiting for a *Bool* message to be used in the *‘clb’*. It will then publish information of *FixedChoice* type to the */scheme* topic. The node is called *‘demonstration’* as for the name of the executable.
In the *‘clb’*, it uses data as an input, and when looking at the *Bool.msg* file on the internet, in the *std_msgs*, it can be understood that to get the Boolean value, the following statement is used:

data.data

Then, in this callback function, the information is used with a simple *if, else* statement, that will assign scheme 1 or 2 according to the result of the Boolean. If it is set to *True*, it would set the global variable *choix* to 1, and 2 in the other case. When exiting that callback function, it will comeback to the *‘talker’* function and send the messages to the desired topic, with an initial value being the scheme 2. This is important, as the initial value for the *‘new_test’* node is also the second scheme as explained in *New_test.py* and therefore it will stop the continous movement of the arm (opening and closing the gripper).
For the *‘talker’* function, it will publish the information to the published topic */scheme* as follows:

while not rospy.is_shutdown(): 
		pub.publish(choix)
		r.sleep()
    
It sleeps following a 1Hz rate, defined a bit before in the code.

When used along with the previous nodes, this whole script achieved to work. The team just had to publish messages on the topic */choose_archi* of the following type:
rostopic pub /scheme std_msgs/Bool “{choice: true}”
