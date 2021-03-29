;; Auto-generated. Do not edit!


(when (boundp 'uvc::enable)
  (if (not (find-package "UVC"))
    (make-package "UVC"))
  (shadow 'enable (find-package "UVC")))
(unless (find-package "UVC::ENABLE")
  (make-package "UVC::ENABLE"))

(in-package "ROS")
;;//! \htmlinclude enable.msg.html


(defclass uvc::enable
  :super ros::object
  :slots (_enable ))

(defmethod uvc::enable
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

(setf (get uvc::enable :md5sum-) "8c1211af706069c994c06e00eb59ac9e")
(setf (get uvc::enable :datatype-) "uvc/enable")
(setf (get uvc::enable :definition-)
      "bool enable
")



(provide :uvc/enable "8c1211af706069c994c06e00eb59ac9e")


