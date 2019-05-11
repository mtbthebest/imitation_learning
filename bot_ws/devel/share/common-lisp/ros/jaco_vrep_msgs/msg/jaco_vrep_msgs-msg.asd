
(cl:in-package :asdf)

(defsystem "jaco_vrep_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Torques" :depends-on ("_package_Torques"))
    (:file "_package_Torques" :depends-on ("_package"))
  ))