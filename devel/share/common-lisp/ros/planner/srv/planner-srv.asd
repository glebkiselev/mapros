
(cl:in-package :asdf)

(defsystem "planner-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AddRobotMessage" :depends-on ("_package_AddRobotMessage"))
    (:file "_package_AddRobotMessage" :depends-on ("_package"))
    (:file "AddTwoInts" :depends-on ("_package_AddTwoInts"))
    (:file "_package_AddTwoInts" :depends-on ("_package"))
  ))