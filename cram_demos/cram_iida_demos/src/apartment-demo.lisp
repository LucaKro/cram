;;;
;;; Copyright (c) 2023, Vanessa Hassouna <hassouna@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors
;;;       may be used to endorse or promote products derived from this software
;;;       without specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :demos)

(defparameter *apartment-object-spawning-poses*
  '((:jeroen-cup
     "island_countertop"
     ((0.3 -0.3 0.1) (0 0 0 1)))
    (:cup
     "cabinet1_coloksu_level4"
     ((0.15 -0.1 0.08) (0 0 -1 0)))
    (:bowl
     "island_countertop"
     ((0.34 0.5 0.072) (0 0 0 -1)))))



(defun initialize-apartment ()
  (sb-ext:gc :full t)

  ;;(when ccl::*is-logging-enabled*
  ;;    (setf ccl::*is-client-connected* nil)
  ;;    (ccl::connect-to-cloud-logger)
  ;;    (ccl::reset-logged-owl))

  ;; (setf proj-reasoning::*projection-checks-enabled* t)

  (kill-and-detach-all)
  (setf (btr:joint-state (btr:get-environment-object)
                         "cabinet1_door_top_left_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "cabinet7_door_bottom_left_joint")
        0.025
        (btr:joint-state (btr:get-environment-object)
                         "dishwasher_drawer_middle_joint")
        0.0)
  (btr-belief::publish-environment-joint-state
   (btr:joint-states (btr:get-environment-object)))

  (coe:clear-belief)

  (setf desig::*designators* (tg:make-weak-hash-table :weakness :key))

  (btr:clear-costmap-vis-object))

(defun finalize ()
  ;; (setf proj-reasoning::*projection-reasoning-enabled* nil)

  ;;(when ccl::*is-logging-enabled*
  ;;  (ccl::export-log-to-owl "ease_milestone_2018.owl")
  ;;  (ccl::export-belief-state-to-owl "ease_milestone_2018_belief.owl"))
  (sb-ext:gc :full t))




(defun apartment-demo-merged (&key (init t) (step 0))
  ;;urdf-proj:with-simulated-robot
  (setf proj-reasoning::*projection-checks-enabled* nil)
  (setf btr:*visibility-threshold* 0.7)
  (when init
    (initialize-apartment)
      ;; (btr-belief:vary-kitchen-urdf '(("handle_cab1_top_door_joint"
      ;;                                  ((-0.038d0 -0.5d0 -0.08d0)
      ;;                                   (0.706825181105366d0 0.0d0
      ;;                                    0.0d0 0.7073882691671998d0)))))
      ;; (setf btr:*current-bullet-world* (make-instance 'btr:bt-reasoning-world))
      ;; (btr-belief:spawn-world)

      (when cram-projection:*projection-environment*
        (spawn-objects-on-fixed-spots
         :object-types '(:jeroen-cup :bowl)
         :spawning-poses-relative *apartment-object-spawning-poses*)))
  (let* ((?on-counter-top-cup-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 2.37 -3 1.0126)
            (cl-transforms:make-quaternion 0 0 1 0)))

         ;;hardcoded pouring stuff

         (?source-object
           (an object
               (type jeroen-cup)
               (name jeroen-cup-1)))

         (?location-on-island
           (a location
              (on (an object
                      (type surface)
                      (urdf-name island-countertop)
                      (part-of apartment)))
              (side back)
              (range 0.4)
              (range-invert 0.3)
              (for ?source-object)))

         (?target-object
           (an object
               (type bowl)
               (name bowl-1)))

         (?location-on-island-target
           (a location
              (on (an object
                      (type surface)
                      (urdf-name island-countertop)
                      (part-of apartment)))
              (side back)
              (range 0.4)
              (range-invert 0.3)
              (for ?target-object)))

         ;; hard-coded stuff for real-world demo
         (?percieve-blue-cup-pose-pouring
           (cl-transforms-stamped:make-pose-stamped
            cram-tf:*fixed-frame*
            0.0
            (cl-transforms:make-3d-vector 1.6 3.25 0.0)
            (cl-transforms:make-quaternion 0 0 0 1)))

         (?on-counter-top-source-cup-look-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 2.5 2.6 1.0126)
            (cl-transforms:make-quaternion 0 1 0 0)))

         (?on-counter-top-target-cup-look-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 2.8 2.2 1.0)
            (cl-transforms:make-quaternion 0 1 0 0)))
         (?third-cup-park-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 1.6867786693573 2.1811975717544556 0)
            (cl-transforms:make-quaternion 0 0 0 1))))

    ;(apartment-demo :step step)

    ;;pick-up-cup
    (when (<= step 2)

      (exe:perform
       (desig:an action
                 (type navigating)
                 (location (desig:a location
                                    (pose ?percieve-blue-cup-pose-pouring)))))

      (let* ((?source-object-desig
               (desig:an object
                         (type jeroen-cup)
                         (color blue)
                         (name jeroen-cup-1)
                         (location ?location-on-island)))

             (?perceive-source-goal
               `(cpoe:object-in-hand ,(an object
                                          (type jeroen-cup)
                                          (name jeroen-cup-1))))
             (?source-perceived-object-desig
               (cpl:seq
                 (exe:perform
                  (desig:an action
                            (type looking)
                            (target (desig:a location
                                             (pose ?on-counter-top-source-cup-look-pose)))
                            (goal ?perceive-source-goal)))
                 (exe:perform
                  (desig:an action
                            (type detecting)
                            (object ?source-object-desig)
                            (goal ?perceive-source-goal)))))

             (?source-grasped-object-desig
               (let ((?goal `(cpoe:object-in-hand ,(an object
                                                       (type jeroen-cup)
                                                       (name jeroen-cup-1)))))
                 (exe:perform
                  (desig:an action
                            (type fetching)
                            (arms (right))
                            (grasps (front))
                            (object ?source-perceived-object-desig)
                            (robot-location (desig:a location
                                                     (pose ?percieve-blue-cup-pose-pouring)))
                            (look-location (desig:a location
                                                    (pose ?on-counter-top-source-cup-look-pose)))
                            (goal ?goal))))))))
      (when (<= step 3)
        (exe:perform
         (desig:an action
                   (type navigating)
                   (location (desig:a location
                                      (pose ?third-cup-park-pose)))))

        (let* ((?target-object-desig
                 (desig:an object
                           (type bowl)
                           (color red)
                           (name bowl-1)
                           (location ?location-on-island-target)))

               (?target-perceived-object-desig
                 (cpl:seq
                   (exe:perform
                    (desig:an action
                              (type looking)
                              (target (desig:a location
                                               (pose ?on-counter-top-target-cup-look-pose)))))
                   (exe:perform
                    (desig:an action
                              (type detecting)
                              (object ?target-object-desig))))))

	  (let* ((?tilty (query-get-maximum-pouring-angle)))
	    (print ?tilty)
          (exe:perform
           (desig:an action
                     (type pouring-without-retries)
                     (on-object ?target-perceived-object-desig)
                     (arm :right)
                     (configuration :top-right)
                     (wait-duration ?tilty)
		     (tilt-angle ?tilty)))))
        
    ;; (exe:perform
    ;;  (desig:an action
    ;;            (type placing)
    ;;            (arm right)
    ;;            (object ?source-object)
    ;;            (target (a location
    ;;                       (pose ?on-counter-top-cup-pose)))))
    )))


