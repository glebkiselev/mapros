
(cl:in-package :asdf)

(defsystem "planner-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Num" :depends-on ("_package_Num"))
    (:file "_package_Num" :depends-on ("_package"))
    (:file "RobotMsg" :depends-on ("_package_RobotMsg"))
    (:file "_package_RobotMsg" :depends-on ("_package"))
  ))