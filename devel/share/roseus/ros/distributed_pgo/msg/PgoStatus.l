;; Auto-generated. Do not edit!


(when (boundp 'distributed_pgo::PgoStatus)
  (if (not (find-package "DISTRIBUTED_PGO"))
    (make-package "DISTRIBUTED_PGO"))
  (shadow 'PgoStatus (find-package "DISTRIBUTED_PGO")))
(unless (find-package "DISTRIBUTED_PGO::PGOSTATUS")
  (make-package "DISTRIBUTED_PGO::PGOSTATUS"))

(in-package "ROS")
;;//! \htmlinclude PgoStatus.msg.html


(defclass distributed_pgo::PgoStatus
  :super ros::object
  :slots (_sender_robot_id _is_converged _current_grad_norm ))

(defmethod distributed_pgo::PgoStatus
  (:init
   (&key
    ((:sender_robot_id __sender_robot_id) 0)
    ((:is_converged __is_converged) nil)
    ((:current_grad_norm __current_grad_norm) 0.0)
    )
   (send-super :init)
   (setq _sender_robot_id (round __sender_robot_id))
   (setq _is_converged __is_converged)
   (setq _current_grad_norm (float __current_grad_norm))
   self)
  (:sender_robot_id
   (&optional __sender_robot_id)
   (if __sender_robot_id (setq _sender_robot_id __sender_robot_id)) _sender_robot_id)
  (:is_converged
   (&optional (__is_converged :null))
   (if (not (eq __is_converged :null)) (setq _is_converged __is_converged)) _is_converged)
  (:current_grad_norm
   (&optional __current_grad_norm)
   (if __current_grad_norm (setq _current_grad_norm __current_grad_norm)) _current_grad_norm)
  (:serialization-length
   ()
   (+
    ;; int32 _sender_robot_id
    4
    ;; bool _is_converged
    1
    ;; float64 _current_grad_norm
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _sender_robot_id
       (write-long _sender_robot_id s)
     ;; bool _is_converged
       (if _is_converged (write-byte -1 s) (write-byte 0 s))
     ;; float64 _current_grad_norm
       (sys::poke _current_grad_norm (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _sender_robot_id
     (setq _sender_robot_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _is_converged
     (setq _is_converged (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _current_grad_norm
     (setq _current_grad_norm (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get distributed_pgo::PgoStatus :md5sum-) "60f16a25306673f726276922d7211a68")
(setf (get distributed_pgo::PgoStatus :datatype-) "distributed_pgo/PgoStatus")
(setf (get distributed_pgo::PgoStatus :definition-)
      "int32 sender_robot_id
bool is_converged
float64 current_grad_norm
")



(provide :distributed_pgo/PgoStatus "60f16a25306673f726276922d7211a68")


