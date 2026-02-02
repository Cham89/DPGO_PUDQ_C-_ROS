; Auto-generated. Do not edit!


(cl:in-package distributed_pgo-msg)


;//! \htmlinclude PgoUpdate.msg.html

(cl:defclass <PgoUpdate> (roslisp-msg-protocol:ros-message)
  ((sender_robot_id
    :reader sender_robot_id
    :initarg :sender_robot_id
    :type cl:integer
    :initform 0)
   (local_vertex_index
    :reader local_vertex_index
    :initarg :local_vertex_index
    :type cl:integer
    :initform 0)
   (pose_pudq
    :reader pose_pudq
    :initarg :pose_pudq
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (pose_vec3
    :reader pose_vec3
    :initarg :pose_vec3
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass PgoUpdate (<PgoUpdate>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PgoUpdate>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PgoUpdate)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name distributed_pgo-msg:<PgoUpdate> is deprecated: use distributed_pgo-msg:PgoUpdate instead.")))

(cl:ensure-generic-function 'sender_robot_id-val :lambda-list '(m))
(cl:defmethod sender_robot_id-val ((m <PgoUpdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader distributed_pgo-msg:sender_robot_id-val is deprecated.  Use distributed_pgo-msg:sender_robot_id instead.")
  (sender_robot_id m))

(cl:ensure-generic-function 'local_vertex_index-val :lambda-list '(m))
(cl:defmethod local_vertex_index-val ((m <PgoUpdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader distributed_pgo-msg:local_vertex_index-val is deprecated.  Use distributed_pgo-msg:local_vertex_index instead.")
  (local_vertex_index m))

(cl:ensure-generic-function 'pose_pudq-val :lambda-list '(m))
(cl:defmethod pose_pudq-val ((m <PgoUpdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader distributed_pgo-msg:pose_pudq-val is deprecated.  Use distributed_pgo-msg:pose_pudq instead.")
  (pose_pudq m))

(cl:ensure-generic-function 'pose_vec3-val :lambda-list '(m))
(cl:defmethod pose_vec3-val ((m <PgoUpdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader distributed_pgo-msg:pose_vec3-val is deprecated.  Use distributed_pgo-msg:pose_vec3 instead.")
  (pose_vec3 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PgoUpdate>) ostream)
  "Serializes a message object of type '<PgoUpdate>"
  (cl:let* ((signed (cl:slot-value msg 'sender_robot_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'local_vertex_index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'pose_pudq))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'pose_vec3))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PgoUpdate>) istream)
  "Deserializes a message object of type '<PgoUpdate>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sender_robot_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'local_vertex_index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:setf (cl:slot-value msg 'pose_pudq) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'pose_pudq)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'pose_vec3) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'pose_vec3)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PgoUpdate>)))
  "Returns string type for a message object of type '<PgoUpdate>"
  "distributed_pgo/PgoUpdate")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PgoUpdate)))
  "Returns string type for a message object of type 'PgoUpdate"
  "distributed_pgo/PgoUpdate")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PgoUpdate>)))
  "Returns md5sum for a message object of type '<PgoUpdate>"
  "bbcafeecfadfbe48488d6fc36fc38955")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PgoUpdate)))
  "Returns md5sum for a message object of type 'PgoUpdate"
  "bbcafeecfadfbe48488d6fc36fc38955")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PgoUpdate>)))
  "Returns full string definition for message of type '<PgoUpdate>"
  (cl:format cl:nil "int32 sender_robot_id~%int32 local_vertex_index~%float64[4] pose_pudq~%float64[3] pose_vec3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PgoUpdate)))
  "Returns full string definition for message of type 'PgoUpdate"
  (cl:format cl:nil "int32 sender_robot_id~%int32 local_vertex_index~%float64[4] pose_pudq~%float64[3] pose_vec3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PgoUpdate>))
  (cl:+ 0
     4
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pose_pudq) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pose_vec3) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PgoUpdate>))
  "Converts a ROS message object to a list"
  (cl:list 'PgoUpdate
    (cl:cons ':sender_robot_id (sender_robot_id msg))
    (cl:cons ':local_vertex_index (local_vertex_index msg))
    (cl:cons ':pose_pudq (pose_pudq msg))
    (cl:cons ':pose_vec3 (pose_vec3 msg))
))
