; Auto-generated. Do not edit!


(cl:in-package uvc-msg)


;//! \htmlinclude enable.msg.html

(cl:defclass <enable> (roslisp-msg-protocol:ros-message)
  ((enable
    :reader enable
    :initarg :enable
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass enable (<enable>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <enable>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'enable)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uvc-msg:<enable> is deprecated: use uvc-msg:enable instead.")))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <enable>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uvc-msg:enable-val is deprecated.  Use uvc-msg:enable instead.")
  (enable m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <enable>) ostream)
  "Serializes a message object of type '<enable>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <enable>) istream)
  "Deserializes a message object of type '<enable>"
    (cl:setf (cl:slot-value msg 'enable) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<enable>)))
  "Returns string type for a message object of type '<enable>"
  "uvc/enable")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'enable)))
  "Returns string type for a message object of type 'enable"
  "uvc/enable")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<enable>)))
  "Returns md5sum for a message object of type '<enable>"
  "8c1211af706069c994c06e00eb59ac9e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'enable)))
  "Returns md5sum for a message object of type 'enable"
  "8c1211af706069c994c06e00eb59ac9e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<enable>)))
  "Returns full string definition for message of type '<enable>"
  (cl:format cl:nil "bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'enable)))
  "Returns full string definition for message of type 'enable"
  (cl:format cl:nil "bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <enable>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <enable>))
  "Converts a ROS message object to a list"
  (cl:list 'enable
    (cl:cons ':enable (enable msg))
))
