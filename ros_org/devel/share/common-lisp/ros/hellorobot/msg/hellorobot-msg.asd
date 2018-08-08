
(cl:in-package :asdf)

(defsystem "hellorobot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "robotMsg" :depends-on ("_package_robotMsg"))
    (:file "_package_robotMsg" :depends-on ("_package"))
  ))