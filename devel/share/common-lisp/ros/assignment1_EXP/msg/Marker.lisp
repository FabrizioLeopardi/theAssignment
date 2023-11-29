; Auto-generated. Do not edit!


(cl:in-package assignment1_EXP-msg)


;//! \htmlinclude Marker.msg.html

(cl:defclass <Marker> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (corners
    :reader corners
    :initarg :corners
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass Marker (<Marker>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Marker>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Marker)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assignment1_EXP-msg:<Marker> is deprecated: use assignment1_EXP-msg:Marker instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <Marker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment1_EXP-msg:id-val is deprecated.  Use assignment1_EXP-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'corners-val :lambda-list '(m))
(cl:defmethod corners-val ((m <Marker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment1_EXP-msg:corners-val is deprecated.  Use assignment1_EXP-msg:corners instead.")
  (corners m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Marker>) ostream)
  "Serializes a message object of type '<Marker>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'corners))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'corners))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Marker>) istream)
  "Deserializes a message object of type '<Marker>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'corners) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'corners)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Marker>)))
  "Returns string type for a message object of type '<Marker>"
  "assignment1_EXP/Marker")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Marker)))
  "Returns string type for a message object of type 'Marker"
  "assignment1_EXP/Marker")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Marker>)))
  "Returns md5sum for a message object of type '<Marker>"
  "29e916ca9398fbd7b2424ca3f887845d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Marker)))
  "Returns md5sum for a message object of type 'Marker"
  "29e916ca9398fbd7b2424ca3f887845d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Marker>)))
  "Returns full string definition for message of type '<Marker>"
  (cl:format cl:nil "int64 id~%geometry_msgs/Point[] corners~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Marker)))
  "Returns full string definition for message of type 'Marker"
  (cl:format cl:nil "int64 id~%geometry_msgs/Point[] corners~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Marker>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'corners) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Marker>))
  "Converts a ROS message object to a list"
  (cl:list 'Marker
    (cl:cons ':id (id msg))
    (cl:cons ':corners (corners msg))
))
