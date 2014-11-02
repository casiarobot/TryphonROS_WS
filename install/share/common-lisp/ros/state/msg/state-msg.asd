
(cl:in-package :asdf)

(defsystem "state-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "state" :depends-on ("_package_state"))
    (:file "_package_state" :depends-on ("_package"))
  ))