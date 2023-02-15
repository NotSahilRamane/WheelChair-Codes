
(cl:in-package :asdf)

(defsystem "av_messages-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "object_" :depends-on ("_package_object_"))
    (:file "_package_object_" :depends-on ("_package"))
    (:file "objects" :depends-on ("_package_objects"))
    (:file "_package_objects" :depends-on ("_package"))
  ))