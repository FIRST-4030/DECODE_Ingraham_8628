package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Class with functionality to obtain the current date as a formatted string.
 *
 * @author Edson James
 * @date 11/14/2025
 */

public class FormattedDateGetter {
    @NonNull
    public static String getDateAndTimeAsString() {
        String pattern = "yy_MM_dd_hh_mm_ss";
        SimpleDateFormat simpleDateFormat = new SimpleDateFormat(pattern);

        String date = simpleDateFormat.format(new Date());

        return date;
    }
}
