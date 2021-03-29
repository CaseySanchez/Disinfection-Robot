; Auto-generated. Do not edit!


(cl:in-package uvc-srv)


;//! \htmlinclude set_lamp-request.msg.html

(cl:defclass <set_lamp-request> (roslisp-msg-protocol:ros-message)
  ((active
    :reader active
    :initarg :active
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass set_lamp-request (<set_lamp-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <set_lamp-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'set_lamp-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uvc-srv:<set_lamp-request> is deprecated: use uvc-srv:set_lamp-request instead.")))

(cl:ensure-generic-function 'active-val :lambda-list '(m))
(cl:defmethod active-val ((m <set_lamp-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uvc-srv:active-val is deprecated.  Use uvc-srv:active instead.")
  (active m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <set_lamp-request>) ostream)
  "Serializes a message object of type '<set_lamp-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <set_lamp-request>) istream)
  "Deserializes a message object of type '<set_lamp-request>"
    (cl:setf (cl:slot-value msg 'active) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<set_lamp-request>)))
  "Returns string type for a service object of type '<set_lamp-request>"
  "uvc/set_lampRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_lamp-request)))
  "Returns string type for a service object of type 'set_lamp-request"
  "uvc/set_lampRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<set_lamp-request>)))
  "Returns md5sum for a message object of type '<set_lamp-request>"
  "fa6cae5645c43d390aecc4758f0073c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'set_lamp-request)))
  "Returns md5sum for a message object of type 'set_lamp-request"
  "fa6cae5645c43d390aecc4758f0073c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<set_lamp-request>)))
  "Returns full string definition for message of type '<set_lamp-request>"
  (cl:format cl:nil "bool active~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'set_lamp-request)))
  "Returns full string definition for message of type 'set_lamp-request"
  (cl:format cl:nil "bool active~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <set_lamp-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <set_lamp-request>))
  "Converts a ROS message object to a list"
  (cl:list 'set_lamp-request
    (cl:cons ':active (active msg))
))
;//! \htmlinclude set_lamp-response.msg.html

(cl:defclass <set_lamp-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass set_lamp-response (<set_lamp-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <set_lamp-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'set_lamp-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uvc-srv:<set_lamp-response> is deprecated: use uvc-srv:set_lamp-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <set_lamp-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uvc-srv:status-val is deprecated.  Use uvc-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <set_lamp-response>) ostream)
  "Serializes a message object of type '<set_lamp-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <set_lamp-response>) istream)
  "Deserializes a message object of type '<set_lamp-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<set_lamp-response>)))
  "Returns string type for a service object of type '<set_lamp-response>"
  "uvc/set_lampResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_lamp-response)))
  "Returns string type for a service object of type 'set_lamp-response"
  "uvc/set_lampResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<set_lamp-response>)))
  "Returns md5sum for a message object of type '<set_lamp-response>"
  "fa6cae5645c43d390aecc4758f0073c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'set_lamp-response)))
  "Returns md5sum for a message object of type 'set_lamp-response"
  "fa6cae5645c43d390aecc4758f0073c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<set_lamp-response>)))
  "Returns full string definition for message of type '<set_lamp-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'set_lamp-response)))
  "Returns full string definition for message of type 'set_lamp-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <set_lamp-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <set_lamp-response>))
  "Converts a ROS message object to a list"
  (cl:list 'set_lamp-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'set_lamp)))
  'set_lamp-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'set_lamp)))
  'set_lamp-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_lamp)))
  "Returns string type for a service object of type '<set_lamp>"
  "uvc/set_lamp")