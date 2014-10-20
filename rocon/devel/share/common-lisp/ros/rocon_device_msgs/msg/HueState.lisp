; Auto-generated. Do not edit!


(cl:in-package rocon_device_msgs-msg)


;//! \htmlinclude HueState.msg.html

(cl:defclass <HueState> (roslisp-msg-protocol:ros-message)
  ((on
    :reader on
    :initarg :on
    :type cl:boolean
    :initform cl:nil)
   (xy
    :reader xy
    :initarg :xy
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (hue
    :reader hue
    :initarg :hue
    :type cl:fixnum
    :initform 0)
   (sat
    :reader sat
    :initarg :sat
    :type cl:fixnum
    :initform 0)
   (bri
    :reader bri
    :initarg :bri
    :type cl:fixnum
    :initform 0)
   (ct
    :reader ct
    :initarg :ct
    :type cl:fixnum
    :initform 0)
   (mode
    :reader mode
    :initarg :mode
    :type cl:string
    :initform "")
   (transitiontime
    :reader transitiontime
    :initarg :transitiontime
    :type cl:integer
    :initform 0)
   (color_mode
    :reader color_mode
    :initarg :color_mode
    :type cl:string
    :initform "")
   (reachable
    :reader reachable
    :initarg :reachable
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass HueState (<HueState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HueState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HueState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rocon_device_msgs-msg:<HueState> is deprecated: use rocon_device_msgs-msg:HueState instead.")))

(cl:ensure-generic-function 'on-val :lambda-list '(m))
(cl:defmethod on-val ((m <HueState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rocon_device_msgs-msg:on-val is deprecated.  Use rocon_device_msgs-msg:on instead.")
  (on m))

(cl:ensure-generic-function 'xy-val :lambda-list '(m))
(cl:defmethod xy-val ((m <HueState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rocon_device_msgs-msg:xy-val is deprecated.  Use rocon_device_msgs-msg:xy instead.")
  (xy m))

(cl:ensure-generic-function 'hue-val :lambda-list '(m))
(cl:defmethod hue-val ((m <HueState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rocon_device_msgs-msg:hue-val is deprecated.  Use rocon_device_msgs-msg:hue instead.")
  (hue m))

(cl:ensure-generic-function 'sat-val :lambda-list '(m))
(cl:defmethod sat-val ((m <HueState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rocon_device_msgs-msg:sat-val is deprecated.  Use rocon_device_msgs-msg:sat instead.")
  (sat m))

(cl:ensure-generic-function 'bri-val :lambda-list '(m))
(cl:defmethod bri-val ((m <HueState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rocon_device_msgs-msg:bri-val is deprecated.  Use rocon_device_msgs-msg:bri instead.")
  (bri m))

(cl:ensure-generic-function 'ct-val :lambda-list '(m))
(cl:defmethod ct-val ((m <HueState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rocon_device_msgs-msg:ct-val is deprecated.  Use rocon_device_msgs-msg:ct instead.")
  (ct m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <HueState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rocon_device_msgs-msg:mode-val is deprecated.  Use rocon_device_msgs-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'transitiontime-val :lambda-list '(m))
(cl:defmethod transitiontime-val ((m <HueState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rocon_device_msgs-msg:transitiontime-val is deprecated.  Use rocon_device_msgs-msg:transitiontime instead.")
  (transitiontime m))

(cl:ensure-generic-function 'color_mode-val :lambda-list '(m))
(cl:defmethod color_mode-val ((m <HueState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rocon_device_msgs-msg:color_mode-val is deprecated.  Use rocon_device_msgs-msg:color_mode instead.")
  (color_mode m))

(cl:ensure-generic-function 'reachable-val :lambda-list '(m))
(cl:defmethod reachable-val ((m <HueState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rocon_device_msgs-msg:reachable-val is deprecated.  Use rocon_device_msgs-msg:reachable instead.")
  (reachable m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<HueState>)))
    "Constants for message type '<HueState>"
  '((:NONE . none)
    (:COLOR_LOOP . colorloop)
    (:SELECT . select)
    (:LSELECT . lselect)
    (:HS . hs #hsv color space)
    (:XY . xy #cie color space)
    (:CT . ct #color temperature space))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'HueState)))
    "Constants for message type 'HueState"
  '((:NONE . none)
    (:COLOR_LOOP . colorloop)
    (:SELECT . select)
    (:LSELECT . lselect)
    (:HS . hs #hsv color space)
    (:XY . xy #cie color space)
    (:CT . ct #color temperature space))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HueState>) ostream)
  "Serializes a message object of type '<HueState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'on) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'xy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'xy))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hue)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'hue)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sat)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bri)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ct)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ct)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mode))
  (cl:let* ((signed (cl:slot-value msg 'transitiontime)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'color_mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'color_mode))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'reachable) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HueState>) istream)
  "Deserializes a message object of type '<HueState>"
    (cl:setf (cl:slot-value msg 'on) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'xy) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'xy)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hue)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'hue)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sat)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bri)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ct)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ct)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'transitiontime) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'color_mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'color_mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'reachable) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HueState>)))
  "Returns string type for a message object of type '<HueState>"
  "rocon_device_msgs/HueState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HueState)))
  "Returns string type for a message object of type 'HueState"
  "rocon_device_msgs/HueState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HueState>)))
  "Returns md5sum for a message object of type '<HueState>"
  "7e4ad09b859196f23a5883df58e12e77")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HueState)))
  "Returns md5sum for a message object of type 'HueState"
  "7e4ad09b859196f23a5883df58e12e77")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HueState>)))
  "Returns full string definition for message of type '<HueState>"
  (cl:format cl:nil "#specific effect~%string NONE = none~%#effect mode~%~%#the light will cycle through all hues using the current brightness and saturation settings.~%string COLOR_LOOP =  colorloop ~%# alert mode~%~%# The light is performing one breathe cycle.~%string SELECT = select  ~%# he light is performing breathe cycles for 30 seconds or mode is none~%string LSELECT = lselect ~%~%#color mode~%string HS = hs #hsv color space~%string XY = xy #cie color space~%string CT = ct #color temperature space~%~%#state~%bool on #light on/off flag true:on /false:off~%~%#color coordination in CIE color space ~%#http://developers.meethue.com/coreconcepts.html#color_gets_more_complicated~%float32[] xy #xy ~%~%#color coordination in HSV color space ~%#http://en.wikipedia.org/wiki/HSL_and_HSV~%uint16 hue #h~%uint8 sat #s~%uint8 bri #v~%~%#color temperature  ~%#http://en.wikipedia.org/wiki/Mired~%#capable of 153 (6500K) to 500 (2000K)~%uint16 ct ~%~%#specific effect~%string mode~%~%#transition time~%int32 transitiontime~%~%~%#config~%string color_mode~%bool reachable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HueState)))
  "Returns full string definition for message of type 'HueState"
  (cl:format cl:nil "#specific effect~%string NONE = none~%#effect mode~%~%#the light will cycle through all hues using the current brightness and saturation settings.~%string COLOR_LOOP =  colorloop ~%# alert mode~%~%# The light is performing one breathe cycle.~%string SELECT = select  ~%# he light is performing breathe cycles for 30 seconds or mode is none~%string LSELECT = lselect ~%~%#color mode~%string HS = hs #hsv color space~%string XY = xy #cie color space~%string CT = ct #color temperature space~%~%#state~%bool on #light on/off flag true:on /false:off~%~%#color coordination in CIE color space ~%#http://developers.meethue.com/coreconcepts.html#color_gets_more_complicated~%float32[] xy #xy ~%~%#color coordination in HSV color space ~%#http://en.wikipedia.org/wiki/HSL_and_HSV~%uint16 hue #h~%uint8 sat #s~%uint8 bri #v~%~%#color temperature  ~%#http://en.wikipedia.org/wiki/Mired~%#capable of 153 (6500K) to 500 (2000K)~%uint16 ct ~%~%#specific effect~%string mode~%~%#transition time~%int32 transitiontime~%~%~%#config~%string color_mode~%bool reachable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HueState>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'xy) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     2
     1
     1
     2
     4 (cl:length (cl:slot-value msg 'mode))
     4
     4 (cl:length (cl:slot-value msg 'color_mode))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HueState>))
  "Converts a ROS message object to a list"
  (cl:list 'HueState
    (cl:cons ':on (on msg))
    (cl:cons ':xy (xy msg))
    (cl:cons ':hue (hue msg))
    (cl:cons ':sat (sat msg))
    (cl:cons ':bri (bri msg))
    (cl:cons ':ct (ct msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':transitiontime (transitiontime msg))
    (cl:cons ':color_mode (color_mode msg))
    (cl:cons ':reachable (reachable msg))
))
