
(cl:in-package :asdf)

(defsystem "assignment1_EXP-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Marker" :depends-on ("_package_Marker"))
    (:file "_package_Marker" :depends-on ("_package"))
  ))