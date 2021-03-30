;; Auto-generated. Do not edit!


(when (boundp 'uvc::set_lamp)
  (if (not (find-package "UVC"))
    (make-package "UVC"))
  (shadow 'set_lamp (find-package "UVC")))
(unless (find-package "UVC::SET_LAMP")
  (make-package "UVC::SET_LAMP"))
(unless (find-package "UVC::SET_LAMPREQUEST")
  (make-package "UVC::SET_LAMPREQUEST"))
(unless (find-package "UVC::SET_LAMPRESPONSE")
  (make-package "UVC::SET_LAMPRESPONSE"))

(in-package "ROS")





(defclass uvc::set_lampRequest
  :super ros::object
  :slots (_active ))

(defmethod uvc::set_lampRequest
  (:init
   (&key
    ((:active __active) nil)
    )
   (send-super :init)
   (setq _active __active)
   self)
  (:active
   (&optional __active)
   (if __active (setq _active __active)) _active)
  (:serialization-length
   ()
   (+
    ;; bool _active
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _active
       (if _active (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _active
     (setq _active (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass uvc::set_lampResponse
  :super ros::object
  :slots (_status ))

(defmethod uvc::set_lampResponse
  (:init
   (&key
    ((:status __status) nil)
    )
   (send-super :init)
   (setq _status __status)
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; bool _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _status
       (if _status (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _status
     (setq _status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass uvc::set_lamp
  :super ros::object
  :slots ())

(setf (get uvc::set_lamp :md5sum-) "fa6cae5645c43d390aecc4758f0073c7")
(setf (get uvc::set_lamp :datatype-) "uvc/set_lamp")
(setf (get uvc::set_lamp :request) uvc::set_lampRequest)
(setf (get uvc::set_lamp :response) uvc::set_lampResponse)

(defmethod uvc::set_lampRequest
  (:response () (instance uvc::set_lampResponse :init)))

(setf (get uvc::set_lampRequest :md5sum-) "fa6cae5645c43d390aecc4758f0073c7")
(setf (get uvc::set_lampRequest :datatype-) "uvc/set_lampRequest")
(setf (get uvc::set_lampRequest :definition-)
      "bool active
---
bool status

")

(setf (get uvc::set_lampResponse :md5sum-) "fa6cae5645c43d390aecc4758f0073c7")
(setf (get uvc::set_lampResponse :datatype-) "uvc/set_lampResponse")
(setf (get uvc::set_lampResponse :definition-)
      "bool active
---
bool status

")



(provide :uvc/set_lamp "fa6cae5645c43d390aecc4758f0073c7")


