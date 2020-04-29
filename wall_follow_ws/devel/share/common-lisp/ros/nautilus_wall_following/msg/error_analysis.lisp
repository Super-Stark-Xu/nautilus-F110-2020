; Auto-generated. Do not edit!


(cl:in-package nautilus_wall_following-msg)


;//! \htmlinclude error_analysis.msg.html

(cl:defclass <error_analysis> (roslisp-msg-protocol:ros-message)
  ((avg_error
    :reader avg_error
    :initarg :avg_error
    :type cl:float
    :initform 0.0)
   (max_error
    :reader max_error
    :initarg :max_error
    :type cl:float
    :initform 0.0))
)

(cl:defclass error_analysis (<error_analysis>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <error_analysis>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'error_analysis)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nautilus_wall_following-msg:<error_analysis> is deprecated: use nautilus_wall_following-msg:error_analysis instead.")))

(cl:ensure-generic-function 'avg_error-val :lambda-list '(m))
(cl:defmethod avg_error-val ((m <error_analysis>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nautilus_wall_following-msg:avg_error-val is deprecated.  Use nautilus_wall_following-msg:avg_error instead.")
  (avg_error m))

(cl:ensure-generic-function 'max_error-val :lambda-list '(m))
(cl:defmethod max_error-val ((m <error_analysis>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nautilus_wall_following-msg:max_error-val is deprecated.  Use nautilus_wall_following-msg:max_error instead.")
  (max_error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <error_analysis>) ostream)
  "Serializes a message object of type '<error_analysis>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'avg_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <error_analysis>) istream)
  "Deserializes a message object of type '<error_analysis>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'avg_error) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_error) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<error_analysis>)))
  "Returns string type for a message object of type '<error_analysis>"
  "nautilus_wall_following/error_analysis")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'error_analysis)))
  "Returns string type for a message object of type 'error_analysis"
  "nautilus_wall_following/error_analysis")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<error_analysis>)))
  "Returns md5sum for a message object of type '<error_analysis>"
  "02c9ccf4f42efdb87083aa4f2e65fcfe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'error_analysis)))
  "Returns md5sum for a message object of type 'error_analysis"
  "02c9ccf4f42efdb87083aa4f2e65fcfe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<error_analysis>)))
  "Returns full string definition for message of type '<error_analysis>"
  (cl:format cl:nil "~%float32 avg_error~%float32 max_error~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'error_analysis)))
  "Returns full string definition for message of type 'error_analysis"
  (cl:format cl:nil "~%float32 avg_error~%float32 max_error~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <error_analysis>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <error_analysis>))
  "Converts a ROS message object to a list"
  (cl:list 'error_analysis
    (cl:cons ':avg_error (avg_error msg))
    (cl:cons ':max_error (max_error msg))
))
