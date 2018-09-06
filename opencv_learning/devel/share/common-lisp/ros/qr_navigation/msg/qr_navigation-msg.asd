
(cl:in-package :asdf)

(defsystem "qr_navigation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "qrMsg" :depends-on ("_package_qrMsg"))
    (:file "_package_qrMsg" :depends-on ("_package"))
  ))