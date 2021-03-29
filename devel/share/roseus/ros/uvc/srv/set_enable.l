;; Auto-generated. Do not edit!


(when (boundp 'uvc::set_enable)
  (if (not (find-package "UVC"))
    (make-package "UVC"))
  (shadow 'set_enable (find-package "UVC")))
(unless (find-package "UVC::SET_ENABLE")
  (make-package "UVC::SET_ENABLE"))
(unless (find-package "UVC::SET_ENABLEREQUEST")
  (make-package "UVC::SET_ENABLEREQUEST"))
(unless (find-package "UVC::SET_ENABLERESPONSE")
  (make-package "UVC::SET_ENABLERESPONSE"))

(in-package "ROS")





(defclass uvc::set_enableRequest
  :super ros::object
  :slots (_enable ))

(defmethod uvc::set_enableRequest
  (:init
   (&key
    ((:enable __enable) nil)
    )
   (send-super :init)
   (setq _enable __enable)
   self)
  (:enable
   (&optional __enable)
   (if __enable (setq _enable __enable)) _enable)
  (:serialization-length
   ()
   (+
    ;; bool _enable
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _enable
       (if _enable (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _enable
     (setq _enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass uvc::set_enableResponse
  :super ros::object
  :slots (_status ))

(defmethod uvc::set_enableResponse
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

(defclass uvc::set_enable
  :super ros::object
  :slots ())

(setf (get uvc::set_enable :md5sum-) "d459cf8faf2747a90df40f12140e9b9c")
(setf (get uvc::set_enable :datatype-) "uvc/set_enable")
(setf (get uvc::set_enable :request) uvc::set_enableRequest)
(setf (get uvc::set_enable :response) uvc::set_enableResponse)

(defmethod uvc::set_enableRequest
  (:response () (instance uvc::set_enableResponse :init)))

(setf (get uvc::set_enableRequest :md5sum-) "d459cf8faf2747a90df40f12140e9b9c")
(setf (get uvc::set_enableRequest :datatype-) "uvc/set_enableRequest")
(setf (get uvc::set_enableRequest :definition-)
      "bool enable
---
bool status

")

(setf (get uvc::set_enableResponse :md5sum-) "d459cf8faf2747a90df40f12140e9b9c")
(setf (get uvc::set_enableResponse :datatype-) "uvc/set_enableResponse")
(setf (get uvc::set_enableResponse :definition-)
      "bool enable
---
bool status

")



(provide :uvc/set_enable "d459cf8faf2747a90df40f12140e9b9c")


