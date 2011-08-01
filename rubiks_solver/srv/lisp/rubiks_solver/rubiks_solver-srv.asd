
(in-package :asdf)

(defsystem "rubiks_solver-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "SolveService" :depends-on ("_package"))
    (:file "_package_SolveService" :depends-on ("_package"))
    ))
