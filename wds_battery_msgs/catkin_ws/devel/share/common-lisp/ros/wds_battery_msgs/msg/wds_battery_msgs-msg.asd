
(cl:in-package :asdf)

(defsystem "wds_battery_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "WdsBattery" :depends-on ("_package_WdsBattery"))
    (:file "_package_WdsBattery" :depends-on ("_package"))
  ))