;; Auto-generated. Do not edit!


(when (boundp 'distributed_pgo::PgoUpdate)
  (if (not (find-package "DISTRIBUTED_PGO"))
    (make-package "DISTRIBUTED_PGO"))
  (shadow 'PgoUpdate (find-package "DISTRIBUTED_PGO")))
(unless (find-package "DISTRIBUTED_PGO::PGOUPDATE")
  (make-package "DISTRIBUTED_PGO::PGOUPDATE"))

(in-package "ROS")
;;//! \htmlinclude PgoUpdate.msg.html


(defclass distributed_pgo::PgoUpdate
  :super ros::object
  :slots (_sender_robot_id _local_vertex_index _pose_pudq _pose_vec3 ))

(defmethod distributed_pgo::PgoUpdate
  (:init
   (&key
    ((:sender_robot_id __sender_robot_id) 0)
    ((:local_vertex_index __local_vertex_index) 0)
    ((:pose_pudq __pose_pudq) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:pose_vec3 __pose_vec3) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _sender_robot_id (round __sender_robot_id))
   (setq _local_vertex_index (round __local_vertex_index))
   (setq _pose_pudq __pose_pudq)
   (setq _pose_vec3 __pose_vec3)
   self)
  (:sender_robot_id
   (&optional __sender_robot_id)
   (if __sender_robot_id (setq _sender_robot_id __sender_robot_id)) _sender_robot_id)
  (:local_vertex_index
   (&optional __local_vertex_index)
   (if __local_vertex_index (setq _local_vertex_index __local_vertex_index)) _local_vertex_index)
  (:pose_pudq
   (&optional __pose_pudq)
   (if __pose_pudq (setq _pose_pudq __pose_pudq)) _pose_pudq)
  (:pose_vec3
   (&optional __pose_vec3)
   (if __pose_vec3 (setq _pose_vec3 __pose_vec3)) _pose_vec3)
  (:serialization-length
   ()
   (+
    ;; int32 _sender_robot_id
    4
    ;; int32 _local_vertex_index
    4
    ;; float64[4] _pose_pudq
    (* 8    4)
    ;; float64[3] _pose_vec3
    (* 8    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _sender_robot_id
       (write-long _sender_robot_id s)
     ;; int32 _local_vertex_index
       (write-long _local_vertex_index s)
     ;; float64[4] _pose_pudq
     (dotimes (i 4)
       (sys::poke (elt _pose_pudq i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _pose_vec3
     (dotimes (i 3)
       (sys::poke (elt _pose_vec3 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _sender_robot_id
     (setq _sender_robot_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _local_vertex_index
     (setq _local_vertex_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64[4] _pose_pudq
   (dotimes (i (length _pose_pudq))
     (setf (elt _pose_pudq i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _pose_vec3
   (dotimes (i (length _pose_vec3))
     (setf (elt _pose_vec3 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get distributed_pgo::PgoUpdate :md5sum-) "bbcafeecfadfbe48488d6fc36fc38955")
(setf (get distributed_pgo::PgoUpdate :datatype-) "distributed_pgo/PgoUpdate")
(setf (get distributed_pgo::PgoUpdate :definition-)
      "int32 sender_robot_id
int32 local_vertex_index
float64[4] pose_pudq
float64[3] pose_vec3

")



(provide :distributed_pgo/PgoUpdate "bbcafeecfadfbe48488d6fc36fc38955")


