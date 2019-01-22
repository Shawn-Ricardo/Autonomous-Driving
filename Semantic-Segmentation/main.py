def load_vgg(sess, vgg_path):
    
    """
    
    Load Pretrained VGG Model into TensorFlow.
    :param sess: TensorFlow Session
    :param vgg_path: Path to vgg folder, containing "variables/" and "saved_model.pb"
    :return: Tuple of Tensors from VGG model (image_input, keep_prob, layer3_out, layer4_out, layer7_out)
    A Fully Convolutional Network has 4 major parts: 1) Encoder, 2) 1x1 Convolutions, 3) Decoder, 
    and 4) Skip Connections.
    
    The encoder will be VGG16, which has been trained on ImageNet.
    
    VGG16 will extract features from the image.
    
    It has been obtained from the following location:
    
    https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/vgg.zip
    
    """
    
    vgg_tag                    = 'vgg16'
    vgg_input_tensor_name      = 'image_input:0'
    vgg_keep_prob_tensor_name  = 'keep_prob:0'
    vgg_layer3_out_tensor_name = 'layer3_out:0'
    vgg_layer4_out_tensor_name = 'layer4_out:0'
    vgg_layer7_out_tensor_name = 'layer7_out:0'

    # load the VGG model into graph variable within this session.
    # saved_model.loader.load() will return a MetaGraphDef protocol buffer
    # that can be used to extract VGG layers. 
    tf.saved_model.loader.load(sess, [vgg_tag], vgg_path)

    # use graph to obtain VGG layers by name. 
    # Grab input layer so the model can take in an image
    vgg_input_layer = sess.graph.get_tensor_by_name(vgg_input_tensor_name)
    
    # Grab "keep probability" to control dropout rate in order to prevent overfitting
    vgg_keep_prob   = sess.graph.get_tensor_by_name(vgg_keep_prob_tensor_name)
    
    # Grab layer 3 output. Useful for implementing Skip Layer technique
    vgg_layer_3_out  = sess.graph.get_tensor_by_name(vgg_layer3_out_tensor_name)
    
    # Grab layer 4 output. Useful for implementing Skip Layer technique
    vgg_layer_4_out  = sess.graph.get_tensor_by_name(vgg_layer4_out_tensor_name)
    
    # Grab layer just prior to implementation of fully connected layers
    # in original VGG16.
    vgg_layer_7_out  = sess.graph.get_tensor_by_name(vgg_layer7_out_tensor_name)
    
    # return these tensors (aka "VGG layers")
    return vgg_input_layer, vgg_keep_prob, vgg_layer_3_out, vgg_layer_4_out, vgg_layer_7_out


test_load_vgg(load_vgg, tf)

def layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes):

    """
    :param vgg_layer7_out: TF Tensor for VGG Layer 3 output
    :param vgg_layer4_out: TF Tensor for VGG Layer 4 output
    :param vgg_layer3_out: TF Tensor for VGG Layer 7 output
    :param num_classes: Number of classes to classify
    :return: The Tensor for the last layer of output
    
    layers() defines the architecture of the Fully Convolutional Network, where 1x1 convolutions,
    decoding, and skip layers are implemented.
    
    the function begins by implementing a 1x1 convolution on the output of vgg_layer7, which is a tensor
    that has extracted numerous features (including spatial features) of the input image. 
    
    In the original implementation of VGG16, a fully connected layer follows layer7 which leads to a 
    softmax classification. For our purposes, the "FCN" starts at this point.
    
    the 1x1 convolution will preserve the tensor shape and allow the network to "rebuild" the original
    image through the process of decoding.
    
    decoding is essentially a reversal of the layers implemented in the forward pass of the convolutional network;
    a process that undoes prior convolutions, all the while preserving the extracted feature information.
    
    skip layers are implemented by connecting the output of a former layer to the output of a non-adjacent 
    latter layer. By doing so, spatial information that is usually lost as data flows through a 
    convolutional network is infused back into the network. In practice, the output of a layer in the encoder
    is added to the output of a layer in the decoder.
    
    As a result, skip layers allow the network to make more precise segmentation decisions.
            
    """
    
    # Implement 1x1 convolution on the output of layer 7 of VGG16 to preserve spatial information
    # and prepare for decoding.
    # Regularization is introduced to reduce overfitting, leading to better model predictions, by
    # restricting the weights from becoming too large.
    conv1x1_1 = tf.layers.conv2d(vgg_layer7_out, num_classes, 
                                 kernel_size=1, 
                                 padding= 'same',                             
                                 kernel_regularizer= tf.contrib.layers.l2_regularizer(1e-3),
                                 kernel_initializer= tf.random_normal_initializer(stddev=0.01))
    
    # decoding is implented through upsampling. In an original convolutional network, the magnitude of 
    # the dimensionality is reduced as information flows through the network.
    # Upsampling returns the data to its original input shape.
    # The value given to 'strides' is what actually performs the upsampling.
    upscale_1 = tf.layers.conv2d_transpose(conv1x1_1, num_classes, 
                                           kernel_size=4, 
                                           strides=2, 
                                           padding= 'same',                                                                                         
                                           kernel_regularizer= tf.contrib.layers.l2_regularizer(1e-3),
                                           kernel_initializer= tf.random_normal_initializer(stddev=0.01))

    # Implement first skip layer.
    # Begin by performing a 1x1 convolution on the output of layer 4. Doing so will transform the shape
    # of the output of vgg layer 4 into the shape of upscale_1, enabling element-wise addition
    conv1x1_2 = tf.layers.conv2d(vgg_layer4_out, num_classes, 
                                 kernel_size=1, 
                                 padding= 'same',                                 
                                 kernel_regularizer= tf.contrib.layers.l2_regularizer(1e-3),
                                 kernel_initializer= tf.random_normal_initializer(stddev=0.01))
    
    # Now that the shapes of the tensors are the same, implement a skip connection    
    skip_connect_1 = tf.add(upscale_1, conv1x1_2)
    
    # Upsample again
    upscale_2 = tf.layers.conv2d_transpose(skip_connect_1, num_classes, 
                                             kernel_size=4,  
                                             strides=2, 
                                             padding= 'same',                                           
                                             kernel_regularizer= tf.contrib.layers.l2_regularizer(1e-3),
                                             kernel_initializer= tf.random_normal_initializer(stddev=0.01))
    

    # Implement second skip layer
    conv1x1_3 = tf.layers.conv2d(vgg_layer3_out, num_classes, 
                                   kernel_size=1, 
                                   padding= 'same',                                 
                                   kernel_regularizer= tf.contrib.layers.l2_regularizer(1e-3),
                                   kernel_initializer= tf.random_normal_initializer(stddev=0.01))
    
    skip_connect_2 = tf.add(upscale_2, conv1x1_3)
    
    # Upsample for the last time, making the shape of the output tensor the same shape as the input image.
    output = tf.layers.conv2d_transpose(skip_connect_2, num_classes, 
                                               kernel_size=16,  
                                               strides=8, 
                                               padding= 'same',                                        
                                               kernel_regularizer= tf.contrib.layers.l2_regularizer(1e-3),
                                               kernel_initializer= tf.random_normal_initializer(stddev=0.01))
    
    # Decoding process complete. return tensor
    return output

test_layers(layers)

def optimize(nn_last_layer, correct_label, learning_rate, num_classes):
    
    """
    :param nn_last_layer: TF Tensor of the last layer in the neural network
    :param correct_label: TF Placeholder for the correct label image
    :param learning_rate: TF Placeholder for the learning rate
    :param num_classes: Number of classes to classify
    :return: Tuple of (logits, train_op, cross_entropy_loss)
    
    The goal of this FCN is to classify each pixel in the image as belonging to the road or not belonging
    to the road. 
    
    To do so, the output of the FCN needs to be reshaped into a 2D tensor, where each row represents
    a pixel and each column represents a class.
    
    When the data is in this 2D state, the model can implement a well-known loss function -- softmax with
    cross-entropy loss -- that can be optimized with another popular optimizer the ADAM Optimizer.
    
    Ultimately, the network will produce a probabilty for a pixel belonging to the road or not. The label with the 
    greatest probability will define that pixel.
    
    Tensorflow.org provides a wonderful discussion on softmax, cross-entropy, and optimization in their 
    Deep-Learning MNIST For Experts tutorial located here...
    https://www.tensorflow.org/get_started/mnist/pros
    
    Optimization and training (look at train_nn()) of this Fully Convolutional Network implement 
    the same steps as in the tutorial.
    
    """

    # resize network output and labels to be same shape
    # logits are the output of the network in the layers() function
    correct_label = tf.reshape(correct_label, (-1,num_classes))
    logits = tf.reshape(nn_last_layer, (-1, num_classes))
    
    # implement softmax with cross entropy as loss function
    cross_entropy = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits= logits, labels= correct_label))
    
    # implement ADAM optimizer to minimize loss. ADAM optimizer will change weights within the network,
    # effectively causing the network to "learn"
    optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate)
    training_operation = optimizer.minimize(cross_entropy)

    return logits, training_operation, cross_entropy

test_optimize(optimize)

def train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, input_image,
             correct_label, keep_prob, learning_rate):
    """
    :param sess: TF Session
    :param epochs: Number of epochs
    :param batch_size: Batch size
    :param get_batches_fn: Function to get batches of training data.  Call using get_batches_fn(batch_size)
    :param train_op: TF Operation to train the neural network
    :param cross_entropy_loss: TF Tensor for the amount of loss
    :param input_image: TF Placeholder for input images
    :param correct_label: TF Placeholder for label images
    :param keep_prob: TF Placeholder for dropout keep probability
    :param learning_rate: TF Placeholder for learning rate
    
    This function trains the FCN.
    
    get_batches_fn() will return N number of images and their corresponding labels from our training
    data, where N = batch_size.
    
    Epochs defines how many times the network trains on the test data set.
    
    The sess parameter will run the network on the optimizer and loss functions. The loss will be 
    printed after each batch is evaluated.
        
    """
    # initialize variables
    sess.run(tf.global_variables_initializer())
    
    for i in range(epochs):
        
        print("EPOCH {} ...".format(i+1))
        
        # get data to train on
        for image, label in get_batches_fn(batch_size):
            
            _, loss = sess.run([train_op, cross_entropy_loss], 
                               feed_dict={input_image: image, correct_label: label,
                               keep_prob: 0.4, learning_rate: 0.001})
            
            print("Loss: = {:.3f}".format(loss))

test_train_nn(train_nn)

def run():

    # we have two classes: road or not roadk
    num_classes = 2
    
    # shape of the input image
    image_shape = (160, 576)
    
    # location of training data
    data_dir = './data'
    
    # location to store model predictions
    runs_dir = './runs'
    
    # define training parameters
    epochs = 25
    batch_size = 10
    # the None parameter tells tensorflow to account for any shape in that dimension
    # set variable names, since model will pass these into functions
    labels = tf.placeholder(tf.int32, [None, None, None, num_classes], name='labels')
    learning_rate = tf.placeholder(tf.float32, name='learning_rate')
    
    # ensure data is training data is present
    tests.test_for_kitti_dataset(data_dir)

    # Download pretrained ("frozen") vgg model
    helper.maybe_download_pretrained_vgg(data_dir)
    
    with tf.Session() as sess:
       
        # set VGG16 path
        vgg16_dir = os.path.join(data_dir, 'vgg')
        
        # Create function to get batches
        get_batches_fn = helper.gen_batch_function(os.path.join(data_dir, 'data_road/training'), image_shape)

        # get VGG16 layers
        input_image, keep_prob, vgg_layer_3_out, vgg_layer_4_out, vgg_layer_7_out = load_vgg(sess, vgg16_dir)

        # get output of FCN
        fcn_output = layers(vgg_layer_3_out, vgg_layer_4_out, vgg_layer_7_out, num_classes)

        # call the optimizer
        logits, train_op, cross_entropy_loss = optimize(fcn_output, labels, learning_rate, num_classes)

        # train the network
        train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, input_image,
             labels, keep_prob, learning_rate)

        # call helper function to test the network.
        # this function will overlay the original image with green on those pixels classified as 
        # belonging to the road...
        helper.save_inference_samples(runs_dir, data_dir, sess, image_shape, logits, keep_prob, input_image)

if __name__ == '__main__':
    run()
