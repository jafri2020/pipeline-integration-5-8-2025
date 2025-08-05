#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from datetime import datetime


seconds = 5 ## Time to be published on /feedback_to_tablet

class KeywordPublisher:
    
    print("keyW_class Started!!!!")
    def __init__(self):
    
        # Initialize the KeywordPublisher class with two publishers
        self.feedback_publisher = rospy.Publisher('/feedback_to_tablet', String, queue_size=10)
        self.command_publisher = rospy.Publisher('/chatbot_command', String, queue_size=10)
        rospy.loginfo("KeywordPublisher initialized. Ready to publish intents and commands.")

    def publish_keyword(self, intent, command):
       
        # Publish messages based on the intent:
        if not rospy.is_shutdown():
            
            # If intent is 'interaction', publish to /feedback_to_table.
            if intent == 'interaction':
                timestamp = datetime.now().strftime('%H:%M:%S')  # Format time as HH:MM:SS
                # message = f"speechonly,{command},seconds"

                message = f"text,{command},{seconds}"
                self.feedback_publisher.publish(message)
                rospy.loginfo(f"Published to /feedback_to_table: {message}")
                
            # If intent is 'control', publish to /chatbot_command.
            elif intent == 'control':
                message = f"{intent},{command}"
                self.command_publisher.publish(message)
                rospy.loginfo(f"Published to /chatbot_command(): {message}")

            else:
                rospy.logwarn(f"Unknown intent: {intent}. No message published.")


if __name__ == '__main__':
    try:
        keyword_publisher = KeywordPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass