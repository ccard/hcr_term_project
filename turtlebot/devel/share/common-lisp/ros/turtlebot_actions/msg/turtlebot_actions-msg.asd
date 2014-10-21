
(cl:in-package :asdf)

(defsystem "turtlebot_actions-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "FindFiducialResult" :depends-on ("_package_FindFiducialResult"))
    (:file "_package_FindFiducialResult" :depends-on ("_package"))
    (:file "FindFiducialAction" :depends-on ("_package_FindFiducialAction"))
    (:file "_package_FindFiducialAction" :depends-on ("_package"))
    (:file "TurtlebotMoveGoal" :depends-on ("_package_TurtlebotMoveGoal"))
    (:file "_package_TurtlebotMoveGoal" :depends-on ("_package"))
    (:file "FindFiducialActionResult" :depends-on ("_package_FindFiducialActionResult"))
    (:file "_package_FindFiducialActionResult" :depends-on ("_package"))
    (:file "TurtlebotMoveResult" :depends-on ("_package_TurtlebotMoveResult"))
    (:file "_package_TurtlebotMoveResult" :depends-on ("_package"))
    (:file "FindFiducialActionGoal" :depends-on ("_package_FindFiducialActionGoal"))
    (:file "_package_FindFiducialActionGoal" :depends-on ("_package"))
    (:file "FindFiducialFeedback" :depends-on ("_package_FindFiducialFeedback"))
    (:file "_package_FindFiducialFeedback" :depends-on ("_package"))
    (:file "TurtlebotMoveActionFeedback" :depends-on ("_package_TurtlebotMoveActionFeedback"))
    (:file "_package_TurtlebotMoveActionFeedback" :depends-on ("_package"))
    (:file "TurtlebotMoveFeedback" :depends-on ("_package_TurtlebotMoveFeedback"))
    (:file "_package_TurtlebotMoveFeedback" :depends-on ("_package"))
    (:file "TurtlebotMoveAction" :depends-on ("_package_TurtlebotMoveAction"))
    (:file "_package_TurtlebotMoveAction" :depends-on ("_package"))
    (:file "FindFiducialGoal" :depends-on ("_package_FindFiducialGoal"))
    (:file "_package_FindFiducialGoal" :depends-on ("_package"))
    (:file "TurtlebotMoveActionGoal" :depends-on ("_package_TurtlebotMoveActionGoal"))
    (:file "_package_TurtlebotMoveActionGoal" :depends-on ("_package"))
    (:file "FindFiducialActionFeedback" :depends-on ("_package_FindFiducialActionFeedback"))
    (:file "_package_FindFiducialActionFeedback" :depends-on ("_package"))
    (:file "TurtlebotMoveActionResult" :depends-on ("_package_TurtlebotMoveActionResult"))
    (:file "_package_TurtlebotMoveActionResult" :depends-on ("_package"))
  ))