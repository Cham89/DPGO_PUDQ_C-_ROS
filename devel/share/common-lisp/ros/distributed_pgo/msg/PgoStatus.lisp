; Auto-generated. Do not edit!


(cl:in-package distributed_pgo-msg)


;//! \htmlinclude PgoStatus.msg.html

(cl:defclass <PgoStatus> (roslisp-msg-protocol:ros-message)
  ((sender_robot_id
    :reader sender_robot_id
    :initarg :sender_robot_id
    :type cl:integer
    :initform 0)
   (is_converged
    :reader is_converged
    :initarg :is_converged
    :type cl:boolean
    :initform cl:nil)
   (current_grad_norm
    :reader current_grad_norm
    :initarg :current_grad_norm
    :type cl:float
    :initform 0.0))
)

(cl:defclass PgoStatus (<PgoStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PgoStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PgoStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name distributed_pgo-msg:<PgoStatus> is deprecated: use distributed_pgo-msg:PgoStatus instead.")))

(cl:ensure-generic-function 'sender_robot_id-val :lambda-list '(m))
(cl:defmethod sender_robot_id-val ((m <PgoStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader distributed_pgo-msg:sender_robot_id-val is deprecated.  Use distributed_pgo-msg:sender_robot_id instead.")
  (sender_robot_id m))

(cl:ensure-generic-function 'is_converged-val :lambda-list '(m))
(cl:defmethod is_converged-val ((m <PgoStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader distributed_pgo-msg:is_converged-val is deprecated.  Use distributed_pgo-msg:is_converged instead.")
  (is_converged m))

(cl:ensure-generic-function 'current_grad_norm-val :lambda-list '(m))
(cl:defmethod current_grad_norm-val ((m <PgoStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader distributed_pgo-msg:current_grad_norm-val is deprecated.  Use distributed_pgo-msg:current_grad_norm instead.")
  (current_grad_norm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PgoStatus>) ostream)
  "Serializes a message object of type '<PgoStatus>"
  (cl:let* ((signed (cl:slot-value msg 'sender_robot_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_converged) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'current_grad_norm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PgoStatus>) istream)
  "Deserializes a message object of type '<PgoStatus>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sender_robot_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'is_converged) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_grad_norm) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PgoStatus>)))
  "Returns string type for a message object of type '<PgoStatus>"
  "distributed_pgo/PgoStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PgoStatus)))
  "Returns string type for a message object of type 'PgoStatus"
  "distributed_pgo/PgoStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PgoStatus>)))
  "Returns md5sum for a message object of type '<PgoStatus>"
  "60f16a25306673f726276922d7211a68")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PgoStatus)))
  "Returns md5sum for a message object of type 'PgoStatus"
  "60f16a25306673f726276922d7211a68")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PgoStatus>)))
  "Returns full string definition for message of type '<PgoStatus>"
  (cl:format cl:nil "int32 sender_robot_id~%bool is_converged~%float64 current_grad_norm~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PgoStatus)))
  "Returns full string definition for message of type 'PgoStatus"
  (cl:format cl:nil "int32 sender_robot_id~%bool is_converged~%float64 current_grad_norm~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PgoStatus>))
  (cl:+ 0
     4
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PgoStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'PgoStatus
    (cl:cons ':sender_robot_id (sender_robot_id msg))
    (cl:cons ':is_converged (is_converged msg))
    (cl:cons ':current_grad_norm (current_grad_norm msg))
))
