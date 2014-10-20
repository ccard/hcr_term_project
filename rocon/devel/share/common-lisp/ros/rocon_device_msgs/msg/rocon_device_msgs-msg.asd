
(cl:in-package :asdf)

(defsystem "rocon_device_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Hue" :depends-on ("_package_Hue"))
    (:file "_package_Hue" :depends-on ("_package"))
    (:file "SetColor" :depends-on ("_package_SetColor"))
    (:file "_package_SetColor" :depends-on ("_package"))
    (:file "HueState" :depends-on ("_package_HueState"))
    (:file "_package_HueState" :depends-on ("_package"))
    (:file "HueArray" :depends-on ("_package_HueArray"))
    (:file "_package_HueArray" :depends-on ("_package"))
  ))