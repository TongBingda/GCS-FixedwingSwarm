import logging, datetime

if __name__ == "__main__":
    logger = logging.getLogger(__file__)
    logger.setLevel(logging.DEBUG)
    # 建立一个filehandler来把日志记录在文件里，级别为debug以上
    logfile_name = datetime.datetime.now().strftime("%Y%m%d-%H-%M-%S") + ".log"
    fh = logging.FileHandler(logfile_name)
    fh.setLevel(logging.DEBUG)
    # 建立一个streamhandler来把日志打在CMD窗口上，级别为debug以上
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    # 设置日志格式
    formatter = logging.Formatter("%(asctime)s: %(message)s")
    ch.setFormatter(formatter)
    fh.setFormatter(formatter)
    #将相应的handler添加在logger对象中
    logger.addHandler(ch)
    logger.addHandler(fh)
    # 开始打日志
    logger.debug("debug message")
    logger.info("info message")
    logger.warn("warn message")
    logger.error("error message")
    logger.critical("critical message")