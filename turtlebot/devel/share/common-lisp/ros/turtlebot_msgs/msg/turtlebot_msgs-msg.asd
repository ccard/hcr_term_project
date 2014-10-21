
(cl:in-package :asdf)

(defsystem "turtlebot_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PanoramaImg" :depends-on ("_package_PanoramaImg"))
    (:file "_package_PanoramaImg" :depends-on ("_package"))
    (:file "LaptopChargeStatus" :depends-on ("_package_LaptopChargeStatus"))
    (:file "_package_LaptopChargeStatus" :depends-on ("_package"))
  ))