#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: exceptions
   :platform: Unix
   :synopsis: Exceptions thrown by rocon's python comms package.

This module defines exceptions raised by the rocon_python_comms package.
These exception names are all included in the main
rocon_python_comms namespace.  To catch one, import it this
way::

    from rocon_python_comms import ServicePairException

----

"""
##############################################################################
# Exceptions
##############################################################################


class ServicePairException(Exception):
    pass


class ServicePairIOException(Exception):
    pass


class NotFoundException(IOError):
    """
      Raised when a requested entity cannot be found, or didn't return the correct result.
    """
    pass
