; Auto-generated. Do not edit!


(cl:in-package bot_gazebo-srv)


;//! \htmlinclude ImageState-request.msg.html

(cl:defclass <ImageState-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ImageState-request (<ImageState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImageState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImageState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bot_gazebo-srv:<ImageState-request> is deprecated: use bot_gazebo-srv:ImageState-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImageState-request>) ostream)
  "Serializes a message object of type '<ImageState-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImageState-request>) istream)
  "Deserializes a message object of type '<ImageState-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImageState-request>)))
  "Returns string type for a service object of type '<ImageState-request>"
  "bot_gazebo/ImageStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageState-request)))
  "Returns string type for a service object of type 'ImageState-request"
  "bot_gazebo/ImageStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImageState-request>)))
  "Returns md5sum for a message object of type '<ImageState-request>"
  "5f88d2cd856d6dd2a610e6e9d5601882")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImageState-request)))
  "Returns md5sum for a message object of type 'ImageState-request"
  "5f88d2cd856d6dd2a610e6e9d5601882")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImageState-request>)))
  "Returns full string definition for message of type '<ImageState-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImageState-request)))
  "Returns full string definition for message of type 'ImageState-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImageState-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImageState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ImageState-request
))
;//! \htmlinclude ImageState-response.msg.html

(cl:defclass <ImageState-response> (roslisp-msg-protocol:ros-message)
  ((saved
    :reader saved
    :initarg :saved
    :type cl:string
    :initform ""))
)

(cl:defclass ImageState-response (<ImageState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImageState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImageState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bot_gazebo-srv:<ImageState-response> is deprecated: use bot_gazebo-srv:ImageState-response instead.")))

(cl:ensure-generic-function 'saved-val :lambda-list '(m))
(cl:defmethod saved-val ((m <ImageState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bot_gazebo-srv:saved-val is deprecated.  Use bot_gazebo-srv:saved instead.")
  (saved m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImageState-response>) ostream)
  "Serializes a message object of type '<ImageState-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'saved))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'saved))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImageState-response>) istream)
  "Deserializes a message object of type '<ImageState-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'saved) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'saved) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImageState-response>)))
  "Returns string type for a service object of type '<ImageState-response>"
  "bot_gazebo/ImageStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageState-response)))
  "Returns string type for a service object of type 'ImageState-response"
  "bot_gazebo/ImageStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImageState-response>)))
  "Returns md5sum for a message object of type '<ImageState-response>"
  "5f88d2cd856d6dd2a610e6e9d5601882")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImageState-response)))
  "Returns md5sum for a message object of type 'ImageState-response"
  "5f88d2cd856d6dd2a610e6e9d5601882")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImageState-response>)))
  "Returns full string definition for message of type '<ImageState-response>"
  (cl:format cl:nil "string saved~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImageState-response)))
  "Returns full string definition for message of type 'ImageState-response"
  (cl:format cl:nil "string saved~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImageState-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'saved))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImageState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ImageState-response
    (cl:cons ':saved (saved msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ImageState)))
  'ImageState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ImageState)))
  'ImageState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageState)))
  "Returns string type for a service object of type '<ImageState>"
  "bot_gazebo/ImageState")