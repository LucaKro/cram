(in-package :btr-tests)

(defun setup-world ()
  (unless roslisp:*master-uri*
    (ros-load:load-system "cram_bullet_reasoning" :cram-bullet-reasoning)
    (ros-load:load-system "cram_pr2_description" :cram-pr2-description)
    (setf roslisp:*master-uri* "http://127.0.0.1:11311"))

  ;;spawn the robot-colliding-objects-without-attached  
  (setf rob-int:*robot-urdf*
        (cl-urdf:parse-urdf (roslisp:get-param "robot_description")))
  (prolog:prolog
   `(and (btr:bullet-world ?world)
         (rob-int:robot ?robot)
         (assert (btr:object ?world :urdf ?robot ((0 0 0) (0 0 0 1))
                             :urdf ,rob-int:*robot-urdf*))
         (assert (btr:joint-state ?world ?robot (("torso_lift_joint" 0.15d0)))))))

(define-test attach-object-unknown-link
  "Tries to attach an item to an unkown link of the robot. This should fail."
  (setup-world)
  (btr-utils:spawn-object 'o1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (lisp-unit:assert-error 'simple-error
                          (btr:attach-object (btr:get-robot-object) (btr:object btr:*current-bullet-world* 'o1)))
  (lisp-unit:assert-error 'simple-error
                          (btr:attach-object (btr:get-robot-object) (btr:object btr:*current-bullet-world* 'o1))
                          :link "asdasdasd1")
  (btr:remove-object btr:*current-bullet-world* 'o1))

(define-test attach-object-same-object-to-two-links
  "Attaches one object to two links of the robot and checks if it is saved properly
in the list under the name of the item attached. The collision information is therefore
shared between the attachments."
  (setup-world)
  (btr-utils:spawn-object 'o1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (let ((link1 "base_link")
        (link2 "base_footprint"))
    (btr:attach-object (btr:get-robot-object) (btr:object btr:*current-bullet-world* 'o1) :link link1)
    (btr:attach-object (btr:get-robot-object) (btr:object btr:*current-bullet-world* 'o1) :link link2)
    ;;'((o1
    ;;   (list of attachments) <- link1, link2 
    ;;   collision-info of o1))
    (lisp-unit:assert-true
     (find link1 (car (cdr
                       (assoc 'o1 (btr:attached-objects (btr:get-robot-object)))))
           :key #'btr::attachment-link :test #'equal))
    (lisp-unit:assert-true
     (find link2 (car (cdr
                       (assoc 'o1 (btr:attached-objects (btr:get-robot-object)))))
           :key #'btr::attachment-link :test #'equal))
    (btr:remove-object btr:*current-bullet-world* 'o1)))

(define-test attach-object-different-objects-to-two-links
  "Attaches two object to two links of the robot and checks if it is saved properly
in the list under the name of the item attached."
  (setup-world)
  (btr-utils:spawn-object 'o1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o2 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (let ((link1 "base_link")
        (link2 "base_footprint"))
    (btr:attach-object (btr:get-robot-object) (btr:object btr:*current-bullet-world* 'o1) :link link1)
    (btr:attach-object (btr:get-robot-object) (btr:object btr:*current-bullet-world* 'o2) :link link2)
    ;;'((o1
    ;;   (list of attachments) <- link1 
    ;;   collision-info of o1)
    ;;  (o2
    ;;   (list of attachments) <- link2
    ;;   collision-info of o2))
    (lisp-unit:assert-true
     (find link1 (car (cdr
                       (assoc 'o1 (btr:attached-objects (btr:get-robot-object)))))
           :key #'btr::attachment-link :test #'equal))
    (lisp-unit:assert-true
     (find link2 (car (cdr
                       (assoc 'o2 (btr:attached-objects (btr:get-robot-object)))))
           :key #'btr::attachment-link :test #'equal))
    (btr:remove-object btr:*current-bullet-world* 'o1)
    (btr:remove-object btr:*current-bullet-world* 'o2)))

(define-test attach-object-different-objects-to-same-link
  "Attaches two objects to one link of the robot and checks if it is saved properly
in the list under the name of the item attached."
  (setup-world)
  (btr-utils:spawn-object 'o1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o2 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (let ((link "base_link"))
    (btr:attach-object (btr:get-robot-object) (btr:object btr:*current-bullet-world* 'o1) :link link)
    (btr:attach-object (btr:get-robot-object) (btr:object btr:*current-bullet-world* 'o2) :link link)
    ;;'((o1
    ;;   (list of attachments) <- link
    ;;   collision-info of o1)
    ;;  (o2
    ;;   (list of attachments) <- link
    ;;   collision-info of o2))
    (lisp-unit:assert-true
     (find link (car (cdr
                       (assoc 'o1 (btr:attached-objects (btr:get-robot-object)))))
           :key #'btr::attachment-link :test #'equal))
    (lisp-unit:assert-true
     (find link (car (cdr
                       (assoc 'o2 (btr:attached-objects (btr:get-robot-object)))))
           :key #'btr::attachment-link :test #'equal))
    (btr:remove-object btr:*current-bullet-world* 'o1)
    (btr:remove-object btr:*current-bullet-world* 'o2)))

(define-test detach-object-completly-same-object-to-two-links
  "Detaches object which was attached to two links of the robot and checks if it is removed properly
in the list under the name of the item attached. Moreover, it should not touch other object attachments."
  (setup-world)
  (btr-utils:spawn-object 'o1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o2 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (let ((link1 "base_link")
        (link2 "base_footprint"))
    (btr:attach-object (btr:get-robot-object) (btr:object btr:*current-bullet-world* 'o1) :link link1)
    (btr:attach-object (btr:get-robot-object) (btr:object btr:*current-bullet-world* 'o1) :link link2)
    (btr:attach-object (btr:get-robot-object) (btr:object btr:*current-bullet-world* 'o2) :link link1)
    ;;'((o1
    ;;   (list of attachments) <- link1 (x), link2 (x) 
    ;;   collision-info of o1)
    ;;  (o2
    ;;   (list of attachments) <- link1
    ;;   collision-info of o2))
    ;;
    ;; marked (x) should be removed
    (btr:detach-object (btr:get-robot-object) (btr:object btr:*current-bullet-world* 'o1))
    (lisp-unit:assert-false
     (assoc 'o1 (btr:attached-objects (btr:get-robot-object))))
    (lisp-unit:assert-true
     (find link1 (car (cdr
                       (assoc 'o2 (btr:attached-objects (btr:get-robot-object)))))
           :key #'btr::attachment-link :test #'equal))
    (btr:remove-object btr:*current-bullet-world* 'o1)
    (btr:remove-object btr:*current-bullet-world* 'o2)))

(define-test detach-object-partially-same-object-to-two-links
  "Detaches object which was attached to two links partially of the robot and checks if it is removed properly
in the list under the name of the item attached. Moreover, it should not touch other object attachments."
  (setup-world)
  (btr-utils:spawn-object 'o1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o2 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (let ((link1 "base_link")
        (link2 "base_footprint"))
    (btr:attach-object (btr:get-robot-object) (btr:object btr:*current-bullet-world* 'o1) :link link1)
    (btr:attach-object (btr:get-robot-object) (btr:object btr:*current-bullet-world* 'o1) :link link2)
    (btr:attach-object (btr:get-robot-object) (btr:object btr:*current-bullet-world* 'o2) :link link1)
    ;;'((o1
    ;;   (list of attachments) <- link1 (x), link2 
    ;;   collision-info of o1)
    ;;  (o2
    ;;   (list of attachments) <- link1
    ;;   collision-info of o2))
    ;;
    ;; marked (x) should be removed
    (btr:detach-object (btr:get-robot-object) (btr:object btr:*current-bullet-world* 'o1) :link link1)
    (lisp-unit:assert-false
     (find link1 (car (cdr
                       (assoc 'o1 (btr:attached-objects (btr:get-robot-object)))))
           :key #'btr::attachment-link :test #'equal))
    (lisp-unit:assert-true
     (find link2 (car (cdr
                       (assoc 'o1 (btr:attached-objects (btr:get-robot-object)))))
           :key #'btr::attachment-link :test #'equal))
    (lisp-unit:assert-true
     (find link1 (car (cdr
                       (assoc 'o2 (btr:attached-objects (btr:get-robot-object)))))
           :key #'btr::attachment-link :test #'equal))
    (btr:remove-object btr:*current-bullet-world* 'o1)
    (btr:remove-object btr:*current-bullet-world* 'o2)))

