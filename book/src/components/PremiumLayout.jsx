import React from 'react';
import clsx from 'clsx';
import { motion } from 'framer-motion';

import styles from './PremiumLayout.module.css';

function PremiumLayout({ children, className }) {
  return (
    <motion.div
      className={clsx(styles.premiumLayout, className)}
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      exit={{ opacity: 0 }}
      transition={{ duration: 0.5 }}
    >
      {children}
    </motion.div>
  );
}

export default PremiumLayout;
