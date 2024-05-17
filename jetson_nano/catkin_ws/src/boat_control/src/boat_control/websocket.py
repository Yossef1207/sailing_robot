import rospy
import socketio
import std_msgs.msg

sio = socketio.Client()

class Websocket():

    #Create the client socket

    def __init__(self):
        
        rospy.init_node("websocket")
        self.subscriber = rospy.Subscriber("/sendViaWebsocket", std_msgs.msg.String, self.send_msg)
        self.log_publisher = rospy.Publisher("/logmsg", std_msgs.msg.String, queue_size=1)

        sio.connect('http://192.168.0.100:3001')
        print("Websocket Node intialized!")
        rospy.spin()

    def send_msg(self, payload):
        sio.emit('gps_coordinates', payload)

    def log(self, msg):
        self.log_publisher.publish(msg)
    
    @sio.event
    def connect():
        print("Websocket-Connection established")

    @sio.event
    def test_event(data):
        #do something in the future...
        #Websocket.log_publisher.publish("Moin!")
        print("Moin2")

def init_websocket():
    websocket = Websocket()
    rospy.spin()
