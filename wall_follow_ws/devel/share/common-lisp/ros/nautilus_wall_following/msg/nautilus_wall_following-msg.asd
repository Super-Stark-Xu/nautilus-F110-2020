
(cl:in-package :asdf)

(defsystem "nautilus_wall_following-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "error_analysis" :depends-on ("_package_error_analysis"))
    (:file "_package_error_analysis" :depends-on ("_package"))
  ))