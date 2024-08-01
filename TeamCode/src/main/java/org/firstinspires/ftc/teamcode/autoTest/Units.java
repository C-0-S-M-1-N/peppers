package org.firstinspires.ftc.teamcode.autoTest;

public enum Units{
    CM{
        @Override
        public double toCM(double value){
            return value;
        }
        @Override
        public double toM(double value){
            return 1e-2 * value;
        }
        @Override
        public double toMM(double value){
            return 1e1 * value;
        }
        @Override
        public double toIN(double value){
            return value * 0.3937;
        }
        @Override
        public double toKM(double value){
            return value * 1e-5;
        }

    },
    MM{
        @Override
        public double toCM(double value){
            return value / 10;
        }
        @Override
        public double toM(double value){
            return value / 1000;
        }
        @Override
        public double toMM(double value){
            return value;
        }
        @Override
        public double toIN(double value){
            return value * 3.937;
        }
        @Override
        public double toKM(double value){
            return value * 1e-6;
        }
    },
    IN(){
        @Override
        public double toCM(double value){
            return value * 2.54;
        }
        @Override
        public double toM(double value){
            return value * 2.54 * 100;
        }
        @Override
        public double toMM(double value){
            return value * 2.54 / 10;
        }
        @Override
        public double toIN(double value){
            return value;
        }
        @Override
        public double toKM(double value){
            return value * 2.54 / 1000;
        }
    },
    M{
        @Override
        public double toCM(double value){
            return value * 100;
        }
        @Override
        public double toM(double value){
            return value;
        }
        @Override
        public double toMM(double value){
            return 1000 * value;
        }
        @Override
        public double toIN(double value){
            return value * 2.54 * 100;
        }
        @Override
        public double toKM(double value){
            return value / 100;
        }
    },
    KM{
        @Override
        public double toCM(double value){
            return value * 1000;
        }
        @Override
        public double toM(double value){
            return value * 100;
        }
        @Override
        public double toMM(double value){
            return value * 1e6;
        }
        @Override
        public double toIN(double value){
            return value * 0.3937 * 1000;
        }
        @Override
        public double toKM(double value){
            return value;
        }
    };
    public double toCM(double value){
        return value;
    }
    public double toM(double value){
        return value;
    }
    public double toMM(double value){
        return value;
    }
    public double toIN(double value){
        return value;
    }
    public double toKM(double value){
        return value;
    }
    public static double conversion(Units from, Units to, double value){
        switch (to) {
            case M:
                return from.toM(value);
            case CM:
                return from.toCM(value);
            case MM:
                return from.toMM(value);
            case KM:
                return from.toKM(value);
            case IN:
                return from.toIN(value);
            default:
                throw new NullPointerException();
        }
    }
}