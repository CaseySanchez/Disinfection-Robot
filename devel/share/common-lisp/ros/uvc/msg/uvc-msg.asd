
(cl:in-package :asdf)

(defsystem "uvc-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "enable" :depends-on ("_package_enable"))
    (:file "_package_enable" :depends-on ("_package"))
  ))