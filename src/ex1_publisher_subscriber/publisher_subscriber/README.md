# Publisher_Subscriber 
In this package there are two file .cpp : 
* publisher.cpp simulates a set of sensors for encoder readings (position of 6 joints). For this reason, it publishes messages with the type "Message1" that is wrapped in the package publisher_subscriber_msgs. 
* subscriber.cpp simulates a set of controllers that prints to stdout messages received by the publisher. 

In order to run the application, there is in the launch directory the file "ex1.launch". 