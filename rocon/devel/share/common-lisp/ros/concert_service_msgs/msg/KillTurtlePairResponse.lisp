; Auto-generated. Do not edit!


(cl:in-package concert_service_msgs-msg)


;//! \htmlinclude KillTurtlePairResponse.msg.html

(cl:defclass <KillTurtlePairResponse> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type uuid_msgs-msg:UniqueID
    :initform (cl:make-instance 'uuid_msgs-msg:UniqueID))
   (response
    :reader response
    :initarg :response
    :type concert_service_msgs-msg:KillTurtleResponse
    :initform (cl:make-instance 'concert_service_msgs-msg:KillTurtleResponse)))
)

(cl:defclass KillTurtlePairResponse (<KillTurtlePairResponse>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <KillTurtlePairResponse>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'KillTurtlePairResponse)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name concert_service_msgs-msg:<KillTurtlePairResponse> is deprecated: use concert_service_msgs-msg:KillTurtlePairResponse instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <KillTurtlePairResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader concert_service_msgs-msg:id-val is deprecated.  Use concert_service_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <KillTurtlePairResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader concert_service_msgs-msg:response-val is deprecated.  Use concert_service_msgs-msg:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <KillTurtlePairResponse>) ostream)
  "Serializes a message object of type '<KillTurtlePairResponse>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'id) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'response) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <KillTurtlePairResponse>) istream)
  "Deserializes a message object of type '<KillTurtlePairResponse>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'id) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'response) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<KillTurtlePairResponse>)))
  "Returns string type for a message object of type '<KillTurtlePairResponse>"
  "concert_service_msgs/KillTurtlePairResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KillTurtlePairResponse)))
  "Returns string type for a message object of type 'KillTurtlePairResponse"
  "concert_service_msgs/KillTurtlePairResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<KillTurtlePairResponse>)))
  "Returns md5sum for a message object of type '<KillTurtlePairResponse>"
  "2d12e3db65db9ae3b7de64597c1f0f15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'KillTurtlePairResponse)))
  "Returns md5sum for a message object of type 'KillTurtlePairResponse"
  "2d12e3db65db9ae3b7de64597c1f0f15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<KillTurtlePairResponse>)))
  "Returns full string definition for message of type '<KillTurtlePairResponse>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM A SERVICE PAIR DEFINITION ======~%~%uuid_msgs/UniqueID id~%KillTurtleResponse response~%~%================================================================================~%MSG: uuid_msgs/UniqueID~%# A universally unique identifier (UUID).~%#~%#  http://en.wikipedia.org/wiki/Universally_unique_identifier~%#  http://tools.ietf.org/html/rfc4122.html~%~%uint8[16] uuid~%~%================================================================================~%MSG: concert_service_msgs/KillTurtleResponse~%# ====== DO NOT MODIFY! AUTOGENERATED FROM A SERVICE PAIR DEFINITION ======~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'KillTurtlePairResponse)))
  "Returns full string definition for message of type 'KillTurtlePairResponse"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM A SERVICE PAIR DEFINITION ======~%~%uuid_msgs/UniqueID id~%KillTurtleResponse response~%~%================================================================================~%MSG: uuid_msgs/UniqueID~%# A universally unique identifier (UUID).~%#~%#  http://en.wikipedia.org/wiki/Universally_unique_identifier~%#  http://tools.ietf.org/html/rfc4122.html~%~%uint8[16] uuid~%~%================================================================================~%MSG: concert_service_msgs/KillTurtleResponse~%# ====== DO NOT MODIFY! AUTOGENERATED FROM A SERVICE PAIR DEFINITION ======~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <KillTurtlePairResponse>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <KillTurtlePairResponse>))
  "Converts a ROS message object to a list"
  (cl:list 'KillTurtlePairResponse
    (cl:cons ':id (id msg))
    (cl:cons ':response (response msg))
))
