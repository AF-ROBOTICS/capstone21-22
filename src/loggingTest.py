from usafalog import *
logger = CreateLogger(__name__)
# if __name__ != "main":
#     pass
# else:
print(logger.getEffectiveLevel())
logger.debug("Debug")
logger.info("info")
logger.warning("WARNING")
logger.error("error")
logger.critical("CRITICAL!")
