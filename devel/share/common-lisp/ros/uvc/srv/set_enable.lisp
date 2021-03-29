; Auto-generated. Do not edit!


(cl:in-package uvc-srv)


;//! \htmlinclude set_enable-request.msg.html

(cl:defclass <set_enable-request> (roslisp-msg-protocol:ros-message)
  ((enable
    :reader enable
    :initarg :enable
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass set_enable-request (<set_enable-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <set_enable-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'set_enable-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uvc-srv:<set_enable-request> is deprecated: use uvc-srv:set_enable-request instead.")))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <set_enable-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uvc-srv:enable-val is deprecated.  Use uvc-srv:enable instead.")
  (enable m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <set_enable-request>) ostream)
  "Serializes a message object of type '<set_enable-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <set_enable-request>) istream)
  "Deserializes a message object of type '<set_enable-request>"
    (cl:setf (cl:slot-value msg 'enable) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<set_enable-request>)))
  "Returns string type for a service object of type '<set_enable-request>"
  "uvc/set_enableRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_enable-request)))
  "Returns string type for a service object of type 'set_enable-request"
  "uvc/set_enableRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<set_enable-request>)))
  "Returns md5sum for a message object of type '<set_enable-request>"
  "d459cf8faf2747a90df40f12140e9b9c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'set_enable-request)))
  "Returns md5sum for a message object of type 'set_enable-request"
  "d459cf8faf2747a90df40f12140e9b9c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<set_enable-request>)))
  "Returns full string definition for message of type '<set_enable-request>"
  (cl:format cl:nil "bool enable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'set_enable-request)))
  "Returns full string definition for message of type 'set_enable-request"
  (cl:format cl:nil "bool enable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <set_enable-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <set_enable-request>))
  "Converts a ROS message object to a list"
  (cl:list 'set_enable-request
    (cl:cons ':enable (enable msg))
))
;//! \htmlinclude set_enable-response.msg.html

(cl:defclass <set_enable-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass set_enable-response (<set_enable-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <set_enable-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'set_enable-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uvc-srv:<set_enable-response> is deprecated: use uvc-srv:set_enable-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <set_enable-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uvc-srv:status-val is deprecated.  Use uvc-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <set_enable-response>) ostream)
  "Serializes a message object of type '<set_enable-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <set_enable-response>) istream)
  "Deserializes a message object of type '<set_enable-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<set_enable-response>)))
  "Returns string type for a service object of type '<set_enable-response>"
  "uvc/set_enableResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_enable-response)))
  "Returns string type for a service object of type 'set_enable-response"
  "uvc/set_enableResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<set_enable-response>)))
  "Returns md5sum for a message object of type '<set_enable-response>"
  "d459cf8faf2747a90df40f12140e9b9c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'set_enable-response)))
  "Returns md5sum for a message object of type 'set_enable-response"
  "d459cf8faf2747a90df40f12140e9b9c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<set_enable-response>)))
  "Returns full string definition for message of type '<set_enable-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'set_enable-response)))
  "Returns full string definition for message of type 'set_enable-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <set_enable-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <set_enable-response>))
  "Converts a ROS message object to a list"
  (cl:list 'set_enable-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'set_enable)))
  'set_enable-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'set_enable)))
  'set_enable-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_enable)))
  "Returns string type for a service object of type '<set_enable>"
  "uvc/set_enable")