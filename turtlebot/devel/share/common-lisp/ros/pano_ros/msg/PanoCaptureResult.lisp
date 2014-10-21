; Auto-generated. Do not edit!


(cl:in-package pano_ros-msg)


;//! \htmlinclude PanoCaptureResult.msg.html

(cl:defclass <PanoCaptureResult> (roslisp-msg-protocol:ros-message)
  ((pano_id
    :reader pano_id
    :initarg :pano_id
    :type cl:integer
    :initform 0)
   (n_captures
    :reader n_captures
    :initarg :n_captures
    :type cl:integer
    :initform 0)
   (bag_filename
    :reader bag_filename
    :initarg :bag_filename
    :type cl:string
    :initform ""))
)

(cl:defclass PanoCaptureResult (<PanoCaptureResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PanoCaptureResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PanoCaptureResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pano_ros-msg:<PanoCaptureResult> is deprecated: use pano_ros-msg:PanoCaptureResult instead.")))

(cl:ensure-generic-function 'pano_id-val :lambda-list '(m))
(cl:defmethod pano_id-val ((m <PanoCaptureResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pano_ros-msg:pano_id-val is deprecated.  Use pano_ros-msg:pano_id instead.")
  (pano_id m))

(cl:ensure-generic-function 'n_captures-val :lambda-list '(m))
(cl:defmethod n_captures-val ((m <PanoCaptureResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pano_ros-msg:n_captures-val is deprecated.  Use pano_ros-msg:n_captures instead.")
  (n_captures m))

(cl:ensure-generic-function 'bag_filename-val :lambda-list '(m))
(cl:defmethod bag_filename-val ((m <PanoCaptureResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pano_ros-msg:bag_filename-val is deprecated.  Use pano_ros-msg:bag_filename instead.")
  (bag_filename m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PanoCaptureResult>) ostream)
  "Serializes a message object of type '<PanoCaptureResult>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pano_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pano_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pano_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pano_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n_captures)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'n_captures)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'n_captures)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'n_captures)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'bag_filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'bag_filename))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PanoCaptureResult>) istream)
  "Deserializes a message object of type '<PanoCaptureResult>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pano_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pano_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pano_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pano_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n_captures)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'n_captures)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'n_captures)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'n_captures)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bag_filename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'bag_filename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PanoCaptureResult>)))
  "Returns string type for a message object of type '<PanoCaptureResult>"
  "pano_ros/PanoCaptureResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PanoCaptureResult)))
  "Returns string type for a message object of type 'PanoCaptureResult"
  "pano_ros/PanoCaptureResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PanoCaptureResult>)))
  "Returns md5sum for a message object of type '<PanoCaptureResult>"
  "7d1ff824dbed21bb16f220c4d06a45fb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PanoCaptureResult)))
  "Returns md5sum for a message object of type 'PanoCaptureResult"
  "7d1ff824dbed21bb16f220c4d06a45fb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PanoCaptureResult>)))
  "Returns full string definition for message of type '<PanoCaptureResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the result~%uint32 pano_id~%uint32 n_captures~%string bag_filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PanoCaptureResult)))
  "Returns full string definition for message of type 'PanoCaptureResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the result~%uint32 pano_id~%uint32 n_captures~%string bag_filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PanoCaptureResult>))
  (cl:+ 0
     4
     4
     4 (cl:length (cl:slot-value msg 'bag_filename))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PanoCaptureResult>))
  "Converts a ROS message object to a list"
  (cl:list 'PanoCaptureResult
    (cl:cons ':pano_id (pano_id msg))
    (cl:cons ':n_captures (n_captures msg))
    (cl:cons ':bag_filename (bag_filename msg))
))
