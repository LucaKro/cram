(in-package :cslg)
(defparameter *mongo-logger* nil)
(defparameter num-experiments 1)
(defparameter connection-retries 0)
(defparameter *start-time* 0)
(defparameter *global-timer* 0)
(defparameter *main-result* '())

(defun prepare-logging ()
  (setf connection-retries 0)
  (print "Cleaning old ros nodes ...")
  ;;(asdf-utils:run-program (concatenate 'string "echo 'y' | rosnode cleanup"))
  (print "Cleaning old ros nodes done")
  (print "Waiting for JSON Prolog to start ...")
  ;;(asdf-utils:run-program (concatenate 'string "python ~/Desktop/lol_launcher.py"))
  (print "JSON Prolog started")
  (setf ccl::*is-client-connected* nil)
  (setf ccl::*is-logging-enabled* t)
  (setf ccl::*host* "'https://localhost'")
  (setf ccl::*cert-path* "'/home/koralewski/Desktop/localhost.pem'")
  ;;(setf ccl::*api-key* "'K103jdr40Rp8UX4egmRf42VbdB1b5PW7qYOOVvTDAoiNG6lcQoaDHONf5KaFcefs'")
  (setf ccl::*api-key* "'uWZv7X1cQkdlZkx4R4uYSIQGyCr4zgyrYEgorw7XjICK0JyLQrmitf48hMCC4pYg'")
  (loop while (and (not ccl::*is-client-connected*) (< connection-retries 10)) do
    (progn
      (ccl::connect-to-cloud-logger)
      (setf connection-retries (+ connection-retries 1))))
  (when (not ccl::*is-client-connected*)
    (/ 1 0)))

(defun call-reset-world-service (id)
  "resets the Unreal world"
  (if (not (roslisp:wait-for-service "/UnrealSim/reset_level" 10))
      (roslisp:ros-warn (send-reset-world-client) "timed out waiting for send-reset-world service")
      (roslisp:call-service "/UnrealSim/reset_level" 'world_control_msgs-srv:ResetLevel :id id)))

(defun main ()
  (ros-load:load-system "cram_pr2_process_modules" :cram-pr2-process-modules)
  (ros-load:load-system "cram_pr2_description" :cram-pr2-description)

  ;;(ros-load:load-system "cram_boxy_description" :cram-boxy-description)
  ;;(setf cram-bullet-reasoning-belief-state:*spawn-debug-window* nil)
  ;; (setf cram-tf:*tf-broadcasting-enabled* t)
  ;; (setf cram-urdf-projection-reasoning::*projection-checks-enabled* nil)
  (roslisp-utilities:startup-ros :name "cram" :anonymous nil)



  ;;(setq roslisp::*debug-stream* nil)
  (print "Init bullet world")
  (dotimes (n 100)
    (progn (setf ccl::*retry-numbers* 0)
           ;; (ccl::start-episode)
           ;; (pr2-pms:with-real-robot (demo::park-robot))
           (cpl:with-failure-handling
            ((common-fail:high-level-failure (e)
                                             (roslisp:ros-warn (pp-plans demo) "Failure happened: ~a~%Skipping..." e)
                                             (return)))
           ;; (pr2-pms:with-real-robot (demo::setting-demo '(:spoon))))
           ;; (pr2-pms:with-real-robot (demo::setting-demo '(:milk))))
           ;; (pr2-pms:with-real-robot (demo::setting-demo '(:bowl)))
           ;; (pr2-pms:with-real-robot (demo::setting-demo '(:cup))))
           (pr2-pms:with-real-robot (demo::setting-demo '(:breakfast-cereal))))
           ;; (pr2-pms:with-real-robot (demo::setting-demo '(:spoon :bowl :milk))))
           ;; (pr2-pms:with-real-robot (demo::setting-demo '(:spoon :bowl :breakfast-cereal :milk))))
           (setf *main-result* (push ccl::*retry-numbers* *main-result*))
           ;; (ccl::stop-episode)
           (call-reset-world-service (write-to-string n))
           (sleep 2)))
  (print *main-result*)
  ;;      do (progn
  ;;           (print "Start")
             ;; (ccl::start-episode)
             ;;(urdf-proj:with-simulated-robot (demo::demo-random nil ))
             ;; ;; (pr2-pms:with-real-robot (demo::setting-demo '(:breakfast-cereal :bowl :spoon :cup :milk)))
  ;;           (pr2-pms:with-real-robot (demo::setting-demo '(:milk)))
             ;; (pr2-pms:with-real-robot (demo::setting-demo '(:breakfast-cereal :milk )))
             ;; (pr2-pms:with-real-robot (demo::setting-demo '(:bowl :spoon :cup :breakfast-cereal :milk)))
             ;; (pr2-pms:with-real-robot (demo::setting-demo '(:milk :breakfast-cereal :bowl :spoon :cup)))
             ;; (pr2-pms:with-real-robot (demo::setting-demo '(:bowl :spoon :cup )))
             ;; (ccl::stop-episode)
  ;;           (print "End")))
  ;; (ccl::finish-logging)
  )
