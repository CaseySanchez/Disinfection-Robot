
(cl:in-package :asdf)

(defsystem "uvc-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "set_enable" :depends-on ("_package_set_enable"))
    (:file "_package_set_enable" :depends-on ("_package"))
    (:file "set_lamp" :depends-on ("_package_set_lamp"))
    (:file "_package_set_lamp" :depends-on ("_package"))
  ))