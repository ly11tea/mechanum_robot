#!/usr/bin/python
# -*- coding: utf-8 -*-
import socket
import time
import rospy
from std_msgs.msg import String
HOST='192.168.1.106'
PORT=1024
s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
s.connect((HOST,PORT))
rospy.init_node('tcptalker',anonymous=0)
pub=rospy.Publisher('tcptopic',String,queue_size=10)
rospy.loginfo("now is listening")   
s.sendall('send\n')   
while not rospy.is_shutdown():
	#cmd=raw_input("Please input cmd:") 
	try:
		s.settimeout(500)          
		data=s.recv(40960)
		pub.publish(data)     
		print data
		print 'once'
		time.sleep(1)
	except socket.timeout:
		print 'time out'
	#s.send('yes i recv\n')    
s.close()   
