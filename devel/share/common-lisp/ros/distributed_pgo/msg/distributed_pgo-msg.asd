
(cl:in-package :asdf)

(defsystem "distributed_pgo-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PgoStatus" :depends-on ("_package_PgoStatus"))
    (:file "_package_PgoStatus" :depends-on ("_package"))
    (:file "PgoUpdate" :depends-on ("_package_PgoUpdate"))
    (:file "_package_PgoUpdate" :depends-on ("_package"))
  ))