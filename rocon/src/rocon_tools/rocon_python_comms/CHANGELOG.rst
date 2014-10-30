Changelog
=========

0.1.9 (2014-08-25)
------------------
* to fix `#54 <https://github.com/robotics-in-concert/rocon_tools/issues/54>`_
* move from symbolic links to includes for changelogs to avoid eclipse bewilderment.
* Contributors: Daniel Stonier, Jihoon Lee

0.1.7 (2014-05-26)
------------------
* update publisher queue_size to avoid warning in indigo.
* robustify find_service against zombie services, closes `#42 <https://github.com/robotics-in-concert/rocon_tools/issues/42>`_.
* updated usage examples for service pairs.
* Contributors: Daniel Stonier

0.1.5 (2014-05-05)
------------------
* sphinx documentation added.
* better feedback on service communication errors.
* comment find_node that it only accepts unresolved names.
* fix missing list return for non-unique find_service requests
* Contributors: Daniel Stonier

0.1.2 (2014-04-02)
------------------
* bugfix location of rospair bash file.
* Contributors: Daniel Stonier

0.1.0 (2014-03-31)
------------------
* bugfix missing list return for non-unique find_topic_requests.
* add find_node function including rostests.
* add find_topic function.
* find_service should catch exceptions when ros shuts down too.
* add find_service function.
* optional threaded callbacks for service pair servers.
* testing against quick calls of the service server.
* wall rate from rocon_utilities.
* subscriber proxy from rocon_utilities.
* rospair command line tool with call, type, list and bash completion.
* rospair command line tool list functionality
* ported in service pairs from multimaster.
* Contributors: Daniel Stonier, Marcus Liebhardt
