(in-package :cslg)
(defparameter *mongo-logger* nil)
(defparameter num-experiments 1)
(defparameter connection-retries 0)
(defparameter *start-time* 0)
(defparameter *global-timer* 0)

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


(defun main ()
  ;;(ros-load:load-system "cram_pr2_description" :cram-pr2-description)
  ;;(ros-load:load-system "cram_boxy_description" :cram-boxy-description)
  ;;(setf cram-bullet-reasoning-belief-state:*spawn-debug-window* nil)
  (setf cram-tf:*tf-broadcasting-enabled* t)
  (setf cram-urdf-projection-reasoning::*projection-checks-enabled* nil)
  (setf ralf-cm::*ralf-on* nil)
  (roslisp-utilities:startup-ros :name "cram" :anonymous nil)
  ;;(setq roslisp::*debug-stream* nil)
  (print "Init bullet world")
  (loop for x from 0 to (- num-experiments 1)
        do (progn
             (print "Start")
             (print (ccl::get-timestamp-for-logging))
             (ccl::start-episode)

             ;; (demos::retail-demo)
             (urdf-proj:with-simulated-robot (demo::setting-demo '(:bowl)))
             ;;(urdf-proj:with-simulated-robot
             ;;  (cpl:with-failure-handling
             ;;      ((condition (e)
             ;;         (print e)
             ;;         (return)))
             ;;    (demo::setting-demo '(:milk)))
             (ccl::stop-episode)
             ;;(print "End"))))
             ;;(ccl::stop-episode)
             ;;(print "End"))))
             (ccl::stop-episode)
             (print (ccl::get-timestamp-for-logging))
             (print "End")))
  (ccl::finish-logging))
