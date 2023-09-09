(in-package :ob-plans)

(defun 2hand-bottle-func (&key
                            ((:collision-mode ?collision-mode))
                            ((:object ?object))
                            ((:object-cap ?object-cap))
                            ((:effort ?grip-effort))
                            ((:right-gripper-opening ?right-gripper-opening))
                            ((:left-gripper-opening ?left-gripper-opening))
                            ((:right-approach-poses ?right-approach-poses))
                            ((:right-grasp-poses ?right-grasp-poses))
                            ((:left-approach-poses ?left-approach-poses))
                            ((:left-grasp-poses ?left-grasp-poses))
                            ((:left-pre-open-poses ?left-pre-open-poses))
                            ((:left-open-poses ?left-open-poses))
                            ((:grasp ?grasp))
                          &allow-other-keys)
  ;;(break)
  (cpl:par
    (roslisp:ros-info (pick-place pick-up) "Opening gripper and reaching")
    (let ((?goal-left `(cpoe:gripper-joint-at :left ,?left-gripper-opening))
          (?goal-right `(cpoe:gripper-joint-at :right ,?right-gripper-opening)))
      (exe:perform
       (desig:an action
                 (type setting-gripper)
                 (gripper :left)
                 (position ?left-gripper-opening)
                 (goal ?goal-left)))
      (exe:perform
       (desig:an action
                 (type setting-gripper)
                 (gripper :right)
                 (position ?right-gripper-opening)
                 (goal ?goal-right))))
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (pp-plans pick-up)
                             "Manipulation messed up: ~a~%Ignoring."
                             e)
           (return)))
      (cpl:par
        ;; this had to be done in parallel so as to specify 2 different objects as goals
        (let ((?goal `(cpoe:tool-frames-at ,?right-approach-poses)))
          (exe:perform
           (desig:an action
                     (type reaching)
                     (object ?object)
                     (right-poses ?right-approach-poses)
                     (goal ?goal))))
        (let ((?goal `(cpoe:tool-frames-at ,?left-approach-poses)))
          (exe:perform
           (desig:an action
                     (type reaching)
                     (object ?object-cap)
                     (left-poses ?left-approach-poses)
                     (goal ?goal)))))))
 ;; (break)
  (roslisp:ros-info (pick-place pick-up) "Grasping")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (cpl:par
      ;; this had to be done in parallel so as to specify 2 different objects as goals
      (let ((?goal `(cpoe:tool-frames-at ,?right-grasp-poses)))
        (exe:perform
         (desig:an action
                   (type grasping)
                   (object ?object)
                   (right-poses ?right-grasp-poses)
                   (goal ?goal))))
      (let ((?goal `(cpoe:tool-frames-at ,?left-grasp-poses)))
        (exe:perform
         (desig:an action
                   (type grasping)
                   (object ?object-cap)
                   (left-poses ?left-grasp-poses)
                   (goal ?goal))))))

  (roslisp:ros-info (pick-place pick-up) "Gripping")

  (cpl:par
    (let ((?goal `(cpoe:object-in-hand ,?object :right)))
      (exe:perform
       (desig:an action
                 (type gripping)
                 (gripper :right)
                 (effort ?grip-effort)
                 (object ?object)
                 (grasp ?grasp)
                 (goal ?goal))))

    (unless (eq ?left-gripper-opening 0.0)
      (let ((?goal `(cpoe:object-in-hand ,?object-cap :left)))
        (exe:perform
         (desig:an action
                   (type gripping)
                   (gripper :left)
                   (effort ?grip-effort)
                   (object ?object-cap)
                   (grasp ?grasp)
                   (goal ?goal))))))
  
  ;; (exe:perform
  ;;  (desig:an action
  ;;            (type approaching)
  ;;            (left-poses ?left-approach-poses)
  ;;            (right-poses ?right-approach-poses)
  ;;            (desig:when ?collision-mode
  ;;              (collision-mode ?collision-mode))))
  ;;(print "lllllllllllllllllll")
  ;;(break)
 
  (mapc
   (lambda (?current-pre-left-open-poses)
     (let ((?poses `(,?current-pre-left-open-poses)))
       (exe:perform
        (desig:an action
                  (type approaching)
                  (left-poses ?poses)
                  (desig:when ?collision-mode
                    (collision-mode ?collision-mode))))))
   ?left-pre-open-poses)
  (btr:detach-object (btr:object btr:*current-bullet-world* :milkbottle-1)
                     (btr:object btr:*current-bullet-world* :milkbottlecap-1))

  (btr:detach-object (btr:object btr:*current-bullet-world* :milkpack-1)
                     (btr:object btr:*current-bullet-world* :milkpackcap-1))

  (btr:detach-object (btr:object btr:*current-bullet-world* :albihimbeerjuice-1)
                     (btr:object btr:*current-bullet-world* :albihimbeerjuicecap-1))
  
  (btr:detach-object (btr:object btr:*current-bullet-world* :winebottle-1)
                     (btr:object btr:*current-bullet-world* :cork-1))
  (btr:attach-object (btr:object btr:*current-bullet-world* :corkscrew-1)
                     (btr:object btr:*current-bullet-world* :cork-1))
  
  (btr:detach-object (btr:object btr:*current-bullet-world* :beerbottle-1)
                     (btr:object btr:*current-bullet-world* :beerbottlecap-1))
  (btr:attach-object (btr:object btr:*current-bullet-world* :caplifter-1)
                     (btr:object btr:*current-bullet-world* :beerbottlecap-1))
  
  ;; (btr:detach-object (btr:object btr:*current-bullet-world* :beerbottle-tall-1)
  ;;                    (btr:object btr:*current-bullet-world* :beerbottlecap-tall-1))
  ;; (btr:attach-object (btr:object btr:*current-bullet-world* :caplifter-1)
  ;;                    (btr:object btr:*current-bullet-world* :beerbottlecap-tall-1))
  (mapc
   (lambda (?current-left-open-poses)
     (let ((?poses `(,?current-left-open-poses)))
       (exe:perform
        (desig:an action
                  (type approaching)
                  (left-poses ?poses)
                  (desig:when ?collision-mode
                    (collision-mode ?collision-mode))))))
   ?left-open-poses)
  ;;(exe:perform (desig:an action (type opening-gripper) (gripper (left))))
  )

(defun name->frame (name)
  (let ((str (string-downcase name)))
    (setf str (substitute #\_ #\- str))
    str))


;; TODO
;; find a way to lookup transform from map to base_footprint
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
