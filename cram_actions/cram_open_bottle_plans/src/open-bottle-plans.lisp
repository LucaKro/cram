(in-package :ob-plans)

(defun open-bottle (&key
                      ((:object ?object))
                      ((:object-cap ?object-cap))
                      ((:tool ?tool))
                      ((:bottle-hand ?bottle-hand))
                      ((:open-hand ?open-hand))
                      ((:grasp ?grasp))
                      ((:effort ?grip-effort))
                      ((:bottle-hand-gripper-opening ?bottle-hand-gripper-opening))
                      ((:open-hand-gripper-opening ?open-hand-gripper-opening))
                      ((:bottle-hand-reach-poses ?bottle-hand-reach-poses))
                      ((:bottle-hand-grasp-poses ?bottle-hand-grasp-poses))
                      ((:open-hand-reach-poses ?open-hand-reach-poses))
                      ((:open-hand-grasp-poses ?open-hand-grasp-poses))
                      ((:open-hand-pre-open-poses ?open-hand-pre-open-poses))
                      ((:open-hand-open-poses ?open-hand-open-poses))
                    &allow-other-keys)
  (cpl:par
    (roslisp:ros-info (pick-place pick-up) "Opening gripper and reaching")
    (let ((?goal-open-hand `(cpoe:gripper-joint-at ?open-hand ,?open-hand-gripper-opening))
          (?goal-bottle-hand `(cpoe:gripper-joint-at ?bottle-hand ,?bottle-hand-gripper-opening)))
      (exe:perform
       (desig:an action
                 (type setting-gripper)
                 (gripper ?open-hand)
                 (position ?open-hand-gripper-opening)
                 (goal ?goal-open-hand)))
      (exe:perform
       (desig:an action
                 (type setting-gripper)
                 (gripper ?bottle-hand)
                 (position ?bottle-hand-gripper-opening)
                 (goal ?goal-bottle-hand))))
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (pp-plans pick-up)
                             "Manipulation messed up: ~a~%Ignoring."
                             e)
           (return)))
      (cpl:par
        ;; this had to be done in parallel so as to specify 2 different objects as goals
        (let ((?goal `(cpoe:tool-frames-at ,?bottle-hand-reach-poses)))
          (exe:perform
           (desig:an action
                     (type reaching)
                     (object ?object)
                     (when (eql ?bottle-hand :right)
                       (right-poses ?bottle-hand-reach-poses))
                     (when (eql ?bottle-hand :left)
                       (left-poses ?bottle-hand-reach-poses))
                     (goal ?goal))))
        (let ((?goal `(cpoe:tool-frames-at ,?open-hand-reach-poses)))
          (exe:perform
           (desig:an action
                     (type reaching)
                     (object ?object-cap)
                     (when (eql ?open-hand :right)
                       (right-poses ?open-hand-reach-poses))
                     (when (eql ?open-hand :left)
                       (left-poses ?open-hand-reach-poses))
                     (goal ?goal)))))))
  (roslisp:ros-info (pick-place pick-up) "Grasping")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (break)
    (cpl:par
      ;; this had to be done in parallel so as to specify 2 different objects as goals
      (let ((?goal `(cpoe:tool-frames-at ,?bottle-hand-grasp-poses)))
        (exe:perform
         (desig:an action
                   (type grasping)
                   (object ?object)
                   (when (eql ?bottle-hand :right)
                       (right-poses ?bottle-hand-grasp-poses))
                   (when (eql ?bottle-hand :left)
                     (left-poses ?bottle-hand-grasp-poses))
                   (goal ?goal))))
      (let ((?goal `(cpoe:tool-frames-at ,?open-hand-grasp-poses)))
        (exe:perform
         (desig:an action
                   (type grasping)
                   (object ?object-cap)
                   (when (eql ?open-hand :right)
                       (right-poses ?open-hand-grasp-poses))
                   (when (eql ?open-hand :left)
                     (left-poses ?open-hand-grasp-poses))
                   (goal ?goal))))))
  (break)
  (roslisp:ros-info (pick-place pick-up) "Gripping")
  (cpl:par
    (let ((?goal `(cpoe:object-in-hand ,?object ?bottle-hand)))
      (exe:perform
       (desig:an action
                 (type gripping)
                 (gripper ?bottle-hand)
                 (effort ?grip-effort)
                 (object ?object)
                 (grasp ?grasp)
                 (goal ?goal))))
    (if (eql ?tool nil)
      (let ((?goal `(cpoe:object-in-hand ,?object-cap ?open-hand)))
        (exe:perform
         (desig:an action
                   (type gripping)
                   (gripper ?open-hand)
                   (effort ?grip-effort)
                   (object ?object-cap)
                   (grasp ?grasp)
                   (goal ?goal))));;)

      (let ((?goal `(cpoe:tool-frames-at ,(last ?open-hand-pre-open-poses))))
        (exe:perform
         (desig:an action
                   (type fixating)
                   (when (eql ?open-hand :right)
                     (right-poses  ?open-hand-pre-open-poses))
                   (when (eql ?open-hand :left)
                     (left-poses  ?open-hand-pre-open-poses))
                   (goal ?goal))))))
  (break)
  (when ?tool
    (btr:attach-object (btr:object btr:*current-bullet-world*
                                   (desig:desig-prop-value ?tool :name))
                       (btr:object btr:*current-bullet-world*
                                   (desig:desig-prop-value ?object-cap :name))))
  ;; (mapc
  ;;  (lambda (?current-open-hand-open-poses)
  ;;    (let ((?poses `(,?current-open-hand-open-poses)))
  ;;      (exe:perform
  ;;       (desig:an action
  ;;                 (type opening)
  ;;                 (when (eql ?open-hand :right)
  ;;                   (right-poses ?poses))
  ;;                 (when (eql ?open-hand :left)
  ;;                   (left-poses ?poses))))))
  ;;  ?open-hand-open-poses)

  (let ((?goal `(cpoe:tool-frames-at ,(last ?open-hand-open-poses))))
    (exe:perform
        (desig:an action
                  (type opening)
                  (when (eql ?open-hand :right)
                    (right-poses ?open-hand-open-poses))
                  (when (eql ?open-hand :left)
                    (left-poses ?open-hand-open-poses))
                  (goal ?goal))))

  ;;(exe:perform (desig:an action (type opening-gripper) (gripper (left))))
  )

(defun name->frame (name)
  (let ((str (string-downcase name)))
    (setf str (substitute #\_ #\- str))
    str))


(defun get-object-lid (object)
  (let* ((bullet-obj (btr:object btr:*current-bullet-world* (desig:desig-prop-value object :name)))
         (bullet-cap (btr:object btr:*current-bullet-world* (car (car (btr:attached-objects bullet-obj))))) (?cap-type (car (btr:item-types bullet-cap)))
         (?cap-name (btr:name bullet-cap))
         (frame (name->frame ?cap-name))
         (pose-in-map (cl-tf:pose->pose-stamped "map" 0 (btr:pose bullet-cap)))
         (transform-in-map (cram-tf:pose-stamped->transform-stamped pose-in-map frame))
         (transform (lookup-transform-using-map pose-in-map (btr:pose (btr:get-robot-object)) "base_footprint" frame))
         (pose (cram-tf:transform->pose-stamped "base_footprint" 0 transform))
         
         (?cap-pose `((:pose ,pose)
                      (:transform ,transform)
                      (:pose-in-map ,pose-in-map)
                      (:transform-in-map ,transform-in-map))))
    (desig:an object
              (type ?cap-type)
              (name ?cap-name)
              (pose ?cap-pose))))

(defun lookup-transform-using-map (pose-stamped1 pose-stamped2 parent-frame child-frame)
  (let ((rel-translation (cl-tf:v- (cl-tf:origin pose-stamped1) (cl-tf:origin pose-stamped2))))
    (multiple-value-bind(angle axis)
        (cl-tf:angle-between-quaternions (cl-tf:orientation pose-stamped1) (cl-tf:orientation pose-stamped2))
      (cl-tf:make-transform-stamped
       parent-frame child-frame 0
       rel-translation
       (cl-tf:axis-angle->quaternion axis angle)))))
