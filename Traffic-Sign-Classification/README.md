# German-Traffic-Sign-Classification
Build a Convolutional Neural Network (CNN) in TensorFlow to classify German traffic signs.

This project works with the German Traffic Sign Dataset (http://benchmark.ini.rub.de/?section=gtsrb&subsection=dataset) which consists of 50,000 colored images of German traffic signs. Each image belongs to 1 of 43 categories, i.e., "stop", "yield", "double curve", "bumpy road", etc.

Convolutional neural networks are a powerful machine learning tool for image classification. With extreme accuracy, they can predict if an object exists in the image or not. Convolutions are the driving force behind this approach. By using a filter and convolving this filter across the surface of the image, a network can learn unique spatial and shape features that are innate to a particule object, such as a car, airplane, or animal. By learning these unqiue features, a CNN can identify, with high accuracy, the object in images that the network has never seen before.

In this project, I train a convolutional nerual network using TensorFlow and explain in great detail the architecture and concepts of a CNN. Training was conducted on an Amazon Web Service EC2 GPU instance. 
