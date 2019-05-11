
(cl:in-package :asdf)

(defsystem "bot_gazebo-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ImageState" :depends-on ("_package_ImageState"))
    (:file "_package_ImageState" :depends-on ("_package"))
  ))